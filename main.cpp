#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

using boost::asio::io_context;
using boost::asio::serial_port;

static constexpr uint8_t SOF0 = 0xAA;
static constexpr uint8_t SOF1 = 0x55;

static constexpr uint8_t TYPE_CAN_DATA = 0x01;
static constexpr uint8_t FRAME_STD = 0x01;
static constexpr uint8_t FRAME_EXT = 0x02;
static constexpr uint8_t FORMAT_DATA = 0x01;

static uint8_t checksum20(const std::array<uint8_t, 20>& p)
{
    uint32_t s = 0;
    for (int i = 2; i <= 18; i++) s += p[i];
    return static_cast<uint8_t>(s & 0xFF);
}

static std::array<uint8_t, 20> buildPacketExt(uint32_t canId29, const uint8_t* data, uint8_t dlc)
{
    std::array<uint8_t, 20> p{};
    p[0] = SOF0;
    p[1] = SOF1;
    p[2] = TYPE_CAN_DATA;
    p[3] = FRAME_EXT;
    p[4] = FORMAT_DATA;

    // little-endian 32-bit id
    p[5] = static_cast<uint8_t>(canId29 & 0xFF);
    p[6] = static_cast<uint8_t>((canId29 >> 8) & 0xFF);
    p[7] = static_cast<uint8_t>((canId29 >> 16) & 0xFF);
    p[8] = static_cast<uint8_t>((canId29 >> 24) & 0xFF);

    p[9] = dlc & 0x0F;

    // DATA bytes, pad with 0xFF
    for (int i = 0; i < 8; i++) {
        p[10 + i] = (i < dlc) ? data[i] : 0xFF;
    }

    p[18] = 0x00;
    p[19] = checksum20(p);
    return p;
}

struct RxFrame
{
    bool extended = false;
    uint32_t id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
};

static bool parsePacket20(const std::array<uint8_t, 20>& p, RxFrame& out)
{
    if (p[0] != SOF0 || p[1] != SOF1) return false;
    if (checksum20(p) != p[19]) return false;

    const uint8_t frame = p[3];
    out.extended = (frame == FRAME_EXT);

    uint32_t id = (uint32_t)p[5]
                | ((uint32_t)p[6] << 8)
                | ((uint32_t)p[7] << 16)
                | ((uint32_t)p[8] << 24);

    out.id  = out.extended ? (id & 0x1FFFFFFF) : (id & 0x7FF);
    out.dlc = p[9] & 0x0F;

    for (int i = 0; i < 8; i++) out.data[i] = p[10 + i];
    return true;
}

// -------- J1939 helpers --------

static uint32_t j1939_id(uint8_t priority, uint32_t pgn, uint8_t sa, uint8_t da_if_pdu1)
{
    uint8_t dp = (pgn >> 16) & 0x01;
    uint8_t pf = (pgn >> 8) & 0xFF;
    uint8_t ps = (pgn >> 0) & 0xFF;

    
    if (pf < 240) ps = da_if_pdu1;

    return ((uint32_t)(priority & 0x7) << 26)
         | ((uint32_t)dp << 24)
         | ((uint32_t)pf << 16)
         | ((uint32_t)ps << 8)
         | (uint32_t)sa;
}

static uint32_t extract_pgn(uint32_t canId29)
{
    uint8_t dp = (canId29 >> 24) & 0x01;
    uint8_t pf = (canId29 >> 16) & 0xFF;
    uint8_t ps = (canId29 >> 8) & 0xFF;

    if (pf < 240) {
        return ((uint32_t)dp << 16) | ((uint32_t)pf << 8);           // DP,PF,00
    } else {
        return ((uint32_t)dp << 16) | ((uint32_t)pf << 8) | ps;      // DP,PF,PS
    }
}

static void dumpFrame(const RxFrame& f)
{
    std::cout << (f.extended ? "EXT " : "STD ")
              << "ID=0x" << std::hex << std::setw(8) << std::setfill('0') << f.id
              << " DLC=" << std::dec << (int)f.dlc << " DATA=";
    for (int i = 0; i < f.dlc; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)f.data[i] << " ";
    }
    std::cout << std::dec << "\n";
}

// -------- TX helpers (ALL TX under one mutex) --------

static void send_frame(serial_port& sp, std::mutex& mtx,
                       uint32_t canId, const uint8_t data[8], uint8_t dlc = 8)
{
    auto pkt = buildPacketExt(canId, data, dlc);

    std::lock_guard<std::mutex> lk(mtx);

    boost::system::error_code ec;
    boost::asio::write(sp, boost::asio::buffer(pkt), ec);
    if (ec) {
       
    }
}

static void send_EEC1(serial_port& sp, std::mutex& mtx,
                      double rpm, double torque_pct,
                      uint8_t sa, uint8_t da = 0xFF, uint8_t prio = 6)
{
    constexpr uint32_t PGN_EEC1 = 0x00F004; // 61444

    uint16_t rawSpeed  = (uint16_t)std::lround(rpm / 0.125);        // rpm = raw * 0.125
    uint8_t  rawTorque = (uint8_t)std::lround(torque_pct + 125.0);  // torque = raw - 125

    uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    d[2] = rawTorque;
    d[3] = (uint8_t)(rawSpeed & 0xFF);
    d[4] = (uint8_t)((rawSpeed >> 8) & 0xFF);

    uint32_t id = j1939_id(prio, PGN_EEC1, sa, da);
    send_frame(sp, mtx, id, d, 8);
}

static void send_ET1(serial_port& sp, std::mutex& mtx,
                     int coolant_c,
                     uint8_t sa, uint8_t da = 0xFF, uint8_t prio = 6)
{
    constexpr uint32_t PGN_ET1 = 0x00FEEE; // 65262

    int raw_i = coolant_c + 40; 
    if (raw_i < 0) raw_i = 0;
    if (raw_i > 250) raw_i = 250;
    uint8_t raw = (uint8_t)raw_i;

    uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    d[0] = raw;

    uint32_t id = j1939_id(prio, PGN_ET1, sa, da);
    send_frame(sp, mtx, id, d, 8);
}

static void send_CCVS(serial_port& sp, std::mutex& mtx,
                      double speed_kmh,
                      uint8_t sa, uint8_t da = 0xFF, uint8_t prio = 6)
{
    constexpr uint32_t PGN_CCVS = 0x00FEF1; // 65265

    // speed = raw * 0.00390625 => raw = speed * 256
    if (speed_kmh < 0) speed_kmh = 0;
    uint16_t raw = (uint16_t)std::lround(speed_kmh * 256.0);

    uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    d[2] = (uint8_t)(raw & 0xFF);
    d[3] = (uint8_t)((raw >> 8) & 0xFF);

    uint32_t id = j1939_id(prio, PGN_CCVS, sa, da);
    send_frame(sp, mtx, id, d, 8);
}

static void send_EEC2(serial_port& sp, std::mutex& mtx,
                      double accel_pct, double load_pct,
                      uint8_t sa, uint8_t da = 0xFF, uint8_t prio = 6)
{
    constexpr uint32_t PGN_EEC2 = 0x00F003; // 61443

    
    if (accel_pct < 0) accel_pct = 0;
    if (accel_pct > 100) accel_pct = 100;
    int rawAccel = (int)std::lround(accel_pct / 0.4);
    if (rawAccel < 0) rawAccel = 0;
    if (rawAccel > 250) rawAccel = 250;

    // load = raw
    if (load_pct < 0) load_pct = 0;
    if (load_pct > 100) load_pct = 100;
    int rawLoad = (int)std::lround(load_pct);

    uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    d[1] = (uint8_t)rawAccel; // Data[1]
    d[2] = (uint8_t)rawLoad;  // Data[2]

    uint32_t id = j1939_id(prio, PGN_EEC2, sa, da);
    send_frame(sp, mtx, id, d, 8);
}

// -------- MAIN --------

int main(int argc, char** argv)
{
    
    std::string portName = (argc >= 2) ? argv[1] : "COM7";

    io_context io;
    serial_port sp(io);

    try {
        sp.open(portName);
        sp.set_option(serial_port::baud_rate(2000000));
        sp.set_option(serial_port::character_size(8));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    } catch (const std::exception& e) {
        std::cerr << "Open serial failed: " << e.what() << "\n";
        return 1;
    }

    std::atomic<bool> run{true};
    std::mutex txMutex;

    // Engine state
    std::atomic<double> rpm{1200.0};
    std::atomic<double> torque_pct{30.0};
    std::atomic<int>    coolant_c{70};
    std::atomic<double> vehicle_kmh{0.0};
    std::atomic<double> accel_pct{0.0};
    std::atomic<double> load_pct{0.0};

    const uint8_t SA_ENGINE = 0x00;
    const uint8_t DA_GLOBAL = 0xFF;
    const uint8_t PRIO = 6;

    // RX thread
    std::thread rxThread([&]{
        std::array<uint8_t, 20> buf{};
        while (run.load()) {
            boost::system::error_code ec;
            size_t n = boost::asio::read(sp, boost::asio::buffer(buf), ec);
            if (ec || n != 20) continue;

            RxFrame fr;
            if (!parsePacket20(buf, fr)) continue;

            dumpFrame(fr);

            if (!fr.extended) continue;

            uint32_t pgn = extract_pgn(fr.id);

            
            if (pgn == 0x00EA00 && fr.dlc >= 3) {
                uint32_t req_pgn = (uint32_t)fr.data[0]
                                 | ((uint32_t)fr.data[1] << 8)
                                 | ((uint32_t)fr.data[2] << 16);
                std::cout << ">> Got REQUEST for PGN 0x" << std::hex << req_pgn << std::dec << "\n";
            }

            
            if (pgn == 0x00EF00 && fr.dlc >= 2) {
                uint8_t cmd = fr.data[0];
                uint8_t arg = fr.data[1];

                if (cmd == 1) rpm.store(800.0 + arg * 10.0);
                if (cmd == 2) torque_pct.store((double)arg);
                std::cout << ">> Control cmd=" << (int)cmd
                          << " rpm=" << rpm.load()
                          << " torque=" << torque_pct.load() << "\n";
            }
        }
    });

    
    std::thread txThread([&]{
        using namespace std::chrono;
        auto t_eec1 = steady_clock::now();
        auto t_et1  = steady_clock::now();
        auto t_eec2 = steady_clock::now();
        auto t_ccvs = steady_clock::now();

        while (run.load()) {
            auto now = steady_clock::now();

            if (now - t_eec1 >= 50ms) {
                t_eec1 = now;
                send_EEC1(sp, txMutex, rpm.load(), torque_pct.load(), SA_ENGINE, DA_GLOBAL, PRIO);
            }

            if (now - t_et1 >= 200ms) {
                t_et1 = now;
                send_ET1(sp, txMutex, coolant_c.load(), SA_ENGINE, DA_GLOBAL, PRIO);
            }

            if (now - t_eec2 >= 100ms) {
                t_eec2 = now;
                send_EEC2(sp, txMutex, accel_pct.load(), load_pct.load(), SA_ENGINE, DA_GLOBAL, PRIO);
            }

            if (now - t_ccvs >= 100ms) {
                t_ccvs = now;
                send_CCVS(sp, txMutex, vehicle_kmh.load(), SA_ENGINE, DA_GLOBAL, PRIO);
            }

            std::this_thread::sleep_for(1ms);
        }
    });

    
    std::thread menuThread([&]{
        while (run.load()) {
            std::cout
                << "\n=== J1939 Engine Simulator Menu ===\n"
                << "1) Обороты двигателя (RPM)\n"
                << "2) Крутящий момент (%)\n"
                << "3) Температура ОЖ (°C)\n"
                << "4) Скорость авто (км/ч)\n"
                << "5) Педаль газа (%)\n"
                << "6) Нагрузка двигателя (%)\n"
                << "0) Выход\n"
                << "Выберите пункт: " << std::flush;

            int choice = -1;
            if (!(std::cin >> choice)) {
                run.store(false);
                return;
            }

            if (choice == 0) {
                run.store(false);
                return;
            }

            std::cout << "Введите значение: " << std::flush;

            double val = 0;
            if (!(std::cin >> val)) {
                run.store(false);
                return;
            }

            switch (choice) {
                case 1: {
                    if (val < 0) val = 0;
                    if (val > 8000) val = 8000;
                    rpm.store(val);
                    send_EEC1(sp, txMutex, rpm.load(), torque_pct.load(), SA_ENGINE, DA_GLOBAL, PRIO);
                    std::cout << "OK: RPM=" << rpm.load() << "\n";
                } break;

                case 2: {
                    if (val < -125) val = -125;
                    if (val > 125)  val = 125;
                    torque_pct.store(val);
                    send_EEC1(sp, txMutex, rpm.load(), torque_pct.load(), SA_ENGINE, DA_GLOBAL, PRIO);
                    std::cout << "OK: Torque%=" << torque_pct.load() << "\n";
                } break;

                case 3: {
                    int t = (int)std::lround(val);
                    if (t < -40) t = -40;
                    if (t > 210) t = 210;
                    coolant_c.store(t);
                    send_ET1(sp, txMutex, coolant_c.load(), SA_ENGINE, DA_GLOBAL, PRIO);
                    std::cout << "OK: Coolant=" << coolant_c.load() << " C\n";
                } break;

                case 4: {
                    if (val < 0) val = 0;
                    if (val > 250) val = 250;
                    vehicle_kmh.store(val);
                    send_CCVS(sp, txMutex, vehicle_kmh.load(), SA_ENGINE, DA_GLOBAL, PRIO);
                    std::cout << "OK: Speed=" << vehicle_kmh.load() << " km/h\n";
                } break;

                case 5: {
                    if (val < 0) val = 0;
                    if (val > 100) val = 100;
                    accel_pct.store(val);
                    send_EEC2(sp, txMutex, accel_pct.load(), load_pct.load(), SA_ENGINE, DA_GLOBAL, PRIO);
                    std::cout << "OK: Pedal=" << accel_pct.load() << " %\n";
                } break;

                case 6: {
                    if (val < 0) val = 0;
                    if (val > 100) val = 100;
                    load_pct.store(val);
                    send_EEC2(sp, txMutex, accel_pct.load(), load_pct.load(), SA_ENGINE, DA_GLOBAL, PRIO);
                    std::cout << "OK: Load=" << load_pct.load() << " %\n";
                } break;

                default:
                    std::cout << "Неизвестный пункт.\n";
                    break;
            }
        }
    });

    std::cout << "J1939 sim running. Use menu (0 = exit).\n";

    // Wait until menu sets run=false
    while (run.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Close port to unblock read
    try { sp.close(); } catch (...) {}

    rxThread.join();
    txThread.join();
    menuThread.join();
    return 0;
}
