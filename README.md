# xspi-8s-crc-retransmission-wk2532
This repository contains Verilog code for a JEDEC JESD251C compliant xSPI controller and slave. It features high-speed (400 MB/s) transfers, dual protocol modes (1S-1S-1S and 8D-8D-8D), and robust CRC8 error checking with retransmission logic. A comprehensive testbench has been developed to verify all functionality.
# xSPI – EXpanded Serial Peripheral Interface (xSPI) for Non Volatile Memory Devices Protocol (Controller + Slave)

## Overview
This repository implements a (xSPI) in Verilog, with both a **controller (master)** and a **slave**.  
It supports command/address/data transactions, CRC checking, and optional retransmission on CRC errors.

The design uses an **8-bit wide IO bus** with shared tri-state-style control (simplified as multiplexed outputs here).  
It is intended for **simulation, testing, or as a template** for building custom synchronous serial protocols.

---

## Top-Level Structure
The design is split into several modules:

| Module | Description |
|--------|-------------|
| **`xspi_top`** | Top-level that instantiates the master (`xspi_sopi_controller`) and slave (`xspi_sopi_slave`), and connects them via a shared IO bus. |
| **`xspi_sopi_controller`** | Implements the **master**: sends command, address, optional write data, and checks received data using CRC8. Handles retransmission on CRC mismatch. |
| **`xspi_sopi_slave`** | Implements the **slave**: receives command/address/data, stores or returns data from a small internal memory register, and responds with CRC8 checks. |
| **`crc8`** | Byte-wise CRC8 generator (polynomial `0x07`) for the master. |
| **`crc8_slave`** | Same as `crc8` but clocked on the slave side (falling edge). |

---

## Key Features
- **8-bit command phase** – Master sends one command byte.
- **48-bit address phase** – Master sends a 6-byte address.
- **64-bit data transfers** – Optional write or read.
- **CRC8 checks** – Separate CRCs for:
  - Command + Address
  - Data payload
- **Error Handling** – Retransmission (up to 3 times) if:
  - Master detects CRC error from slave
  - Slave detects CRC error from master
- **Parameterizable Commands** – Example:
  - `0xFF` → Read
  - `0xA5` → Write

---

## Protocol Timing
A full transaction typically follows this sequence:

1. **Master asserts `CS_n` low**.
2. **Send Command** (1 byte) → CRC update.
3. **Send Address** (6 bytes) → CRC update.
4. **Send CRC for Command+Address**.
5. Depending on command:
   - **Read (`0xFF`)**:
     - Slave waits fixed latency (`Latency = 6 cycles`)
     - Slave sends 8 bytes of data + CRC.
   - **Write (`0xA5`)**:
     - Master sends 8 bytes of data + CRC.
6. **CRC Validation** – Both sides compare CRCs and set error/match flags.
7. **Retransmit if necessary** (up to 3 attempts).

---

## I/O Ports – Top Level (`xspi_top`)
| Signal | Direction | Width | Description |
|--------|-----------|-------|-------------|
| `clk` | in | 1 | System clock |
| `rst_n` | in | 1 | Active-low reset |
| `start` | in | 1 | Start transaction trigger |
| `command` | in | 8 | Command byte |
| `address` | in | 48 | Target address |
| `wr_data` | in | 64 | Data to write (if write command) |
| `rd_data` | out | 64 | Data read from slave |
| `done` | out | 1 | Transaction complete flag |
| `ready` | out | 1 | Slave ready status |
| `crc_*` | out | 1 | CRC match/error flags for debug |

---

## CRC Details
- **Polynomial:** `x^8 + x^2 + x + 1` (`0x07`)
- **Initial value:** `0x00`
- **Byte-at-a-time processing**
- **Separate CRC blocks for:**
  - Command/Address (CA)
  - Data

---

## Retransmission Logic
- Both master and slave maintain a `retransmit_cnt`.
- If CRC mismatch occurs:
  - Retry from the **command phase**.
  - Limit: 3 retransmissions.

---

## Example Usage (Simulation)
You can simulate the protocol using a testbench like:

```verilog
initial begin
    rst_n = 0;
    clk = 0;
    #10 rst_n = 1;
    start = 1;
    command = 8'hA5; // Write
    address = 48'h123456_ABCDEF;
    wr_data = 64'hDEADBEEF_CAFE_BABE;
    #10 start = 0;
end

always #5 clk = ~clk;
