# ReRust on ESP32

Executes a simple reactive program on the ESP32. Custom Rust compiler is needed for the Xtensa architecture. Also, the compiler has to be patched with `eaddc8febd02ca0b0cbd0fa08b8b752cb347b725` for memory allocation to work. See [https://github.com/MabezDev/xtensa-rust-quickstart](https://github.com/MabezDev/xtensa-rust-quickstart) for information on how to setup the toolchain.