#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

using boost::asio::serial_port;
using boost::asio::io_context;

static constexpr uint8_t SOF0 = 0xAA;
static constexpr uint8_t SOF1 = 0x55;

static constexpr uint8_t TYPE_CAN_DATA = 0x01;
static constexpr uint8_t FRAME_STD = 0x01;
static constexpr uint8_t FRAME_EXT = 0x02;
static constexpr uint8_t FORMAT_DATA = 0x01;

static uint8_t checksum20(const std::array<uint8_t, 20>& p) {
    uint32_t s = 0;
    for (int i = 2; i <= 18; i++) s += p[i];
    return static_cast<uint8_t>(s & 0xFF);
}

static std::array<uint8_t, 20> buildPacketExt(uint32_t canId29, const uint8_t* data, uint8_t dlc) {
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
    for (int i = 0; i < 8; i++) p[10 + i] = (i < dlc) ? data[i] : 0xFF;

    p[18] = 0x00;
    p[19] = checksum20(p);
    return p;
}

struct RxFrame {
    bool extended = false;
    uint32_t id = 0;          // 11-bit or 29-bit (we use 29)
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
};

static bool parsePacket20(const std::array<uint8_t, 20>& p, RxFrame& out) {
    if (p[0] != SOF0 || p[1] != SOF1) return false;
    if (checksum20(p) != p[19]) return false;

    const uint8_t frame = p[3];
    out.extended = (frame == FRAME_EXT);

    uint32_t id = (uint32_t)p[5]
                | ((uint32_t)p[6] << 8)
                | ((uint32_t)p[7] << 16)
                | ((uint32_t)p[8] << 24);

    out.id = out.extended ? (id & 0x1FFFFFFF) : (id & 0x7FF);
    out.dlc = p[9] & 0x0F;
    for (int i = 0; i < 8; i++) out.data[i] = p[10 + i];
    return true;
}

// J1939 helpers
static uint32_t j1939_id(uint8_t priority, uint32_t pgn, uint8_t sa, uint8_t da_if_pdu1) {
    // pgn: 18-bit (DP:1 at bit16, PF:8 at bits 8..15, PS:8 at bits 0..7)
    uint8_t dp = (pgn >> 16) & 0x01;
    uint8_t pf = (pgn >> 8) & 0xFF;
    uint8_t ps = (pgn >> 0) & 0xFF;

    // PDU1: PF < 240 -> PS is DA in CAN-ID
    if (pf < 240) ps = da_if_pdu1;

    return ((uint32_t)(priority & 0x7) << 26)
         | ((uint32_t)dp << 24)
         | ((uint32_t)pf << 16)
         | ((uint32_t)ps << 8)
         | (uint32_t)sa;
}

static uint32_t extract_pgn(uint32_t canId29) {
    uint8_t dp = (canId29 >> 24) & 0x01;
    uint8_t pf = (canId29 >> 16) & 0xFF;
    uint8_t ps = (canId29 >> 8) & 0xFF;

    if (pf < 240) {
        // PDU1: PGN = DP,PF,00
        return ((uint32_t)dp << 16) | ((uint32_t)pf << 8);
    } else {
        // PDU2: PGN = DP,PF,PS
        return ((uint32_t)dp << 16) | ((uint32_t)pf << 8) | ps;
    }
}

static void dumpFrame(const RxFrame& f) {
    std::cout << (f.extended ? "EXT " : "STD ")
              << "ID=0x" << std::hex << std::setw(8) << std::setfill('0') << f.id
              << " DLC=" << std::dec << (int)f.dlc << " DATA=";
    for (int i = 0; i < f.dlc; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)f.data[i] << " ";
    }
    std::cout << std::dec << "\n";
}

int main(int argc, char** argv) {
    // Windows: "COM7" ; Linux: "/dev/ttyUSB0"
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

    // Engine state (меняем по входящим "командам")
    std::atomic<double> rpm{1200.0};
    std::atomic<double> torque_pct{30.0};
    std::atomic<int> coolant_c{70};

    // Thread: RX
    std::thread rxThread([&]{
        std::array<uint8_t, 20> buf{};
        while (run.load()) {
            boost::system::error_code ec;
            size_t n = boost::asio::read(sp, boost::asio::buffer(buf), ec);
            if (ec || n != 20) continue;

            RxFrame fr;
            if (!parsePacket20(buf, fr)) continue;

            // Логи
            dumpFrame(fr);

            if (!fr.extended) continue;

            uint32_t pgn = extract_pgn(fr.id);

            // Request PGN 59904 = 0x00EA00
            if (pgn == 0x00EA00 && fr.dlc >= 3) {
                uint32_t req_pgn = (uint32_t)fr.data[0]
                                | ((uint32_t)fr.data[1] << 8)
                                | ((uint32_t)fr.data[2] << 16);
                std::cout << ">> Got REQUEST for PGN 0x" << std::hex << req_pgn << std::dec << "\n";
                // Тут можно сделать ответ на req_pgn (если тебе надо)
            }

            // Пример: если ты хочешь считать "управляющую команду" по какому-то своему PGN,
            // например 0x00EF00 (proprietary), то обработай тут:
            if (pgn == 0x00EF00 && fr.dlc >= 4) {
                // допустим data[0]=cmd, data[1]=arg...
                uint8_t cmd = fr.data[0];
                if (cmd == 1) rpm.store(800 + fr.data[1] * 10.0);
                if (cmd == 2) torque_pct.store(fr.data[1] * 1.0);
                std::cout << ">> Control cmd=" << (int)cmd
                          << " rpm=" << rpm.load()
                          << " torque=" << torque_pct.load() << "\n";
            }
        }
    });

    // Thread: TX циклы
    std::thread txThread([&]{
        using namespace std::chrono;
        auto t_eec1 = steady_clock::now();
        auto t_et1  = steady_clock::now();

        // PGNs
        constexpr uint32_t PGN_EEC1 = 0x00F004; // 61444
        constexpr uint32_t PGN_ET1  = 0x00FEEE; // 65262

        const uint8_t PRIO = 6;
        const uint8_t SA_ENGINE = 0x00; // адрес "двигателя" (в симуляторе)
        const uint8_t DA_GLOBAL = 0xFF; // глобально

        while (run.load()) {
            auto now = steady_clock::now();

            // EEC1 каждые 50ms
            if (now - t_eec1 >= 50ms) {
                t_eec1 = now;

                // как у тебя в плате: rawSpeed = data[3] | data[4]<<8; rpm = rawSpeed*0.125
                uint16_t rawSpeed = (uint16_t)(rpm.load() / 0.125);
                uint8_t rawTorque = (uint8_t)(torque_pct.load() + 125.0); // если используешь смещение -125

                uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
                d[2] = rawTorque;
                d[3] = (uint8_t)(rawSpeed & 0xFF);
                d[4] = (uint8_t)((rawSpeed >> 8) & 0xFF);

                uint32_t id = j1939_id(PRIO, PGN_EEC1, SA_ENGINE, DA_GLOBAL);
                auto pkt = buildPacketExt(id, d, 8);

                boost::system::error_code ec;
                boost::asio::write(sp, boost::asio::buffer(pkt), ec);
            }

            // ET1 каждые 200ms
            if (now - t_et1 >= 200ms) {
                t_et1 = now;
                // J1939 coolant temp обычно byte1 с -40 offset, у тебя было Data[0]
                int tc = coolant_c.load();
                uint8_t raw = (tc <= -40) ? 0 : (uint8_t)(tc + 40);

                uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
                d[0] = raw;

                uint32_t id = j1939_id(6, PGN_ET1, SA_ENGINE, DA_GLOBAL);
                auto pkt = buildPacketExt(id, d, 8);

                boost::system::error_code ec;
                boost::asio::write(sp, boost::asio::buffer(pkt), ec);
            }

            std::this_thread::sleep_for(1ms);
        }
    });

    std::cout << "J1939 sim running. Press Enter to stop...\n";
    std::cin.get();
    run.store(false);

    try { sp.close(); } catch (...) {}
    rxThread.join();
    txThread.join();
    return 0;
}
