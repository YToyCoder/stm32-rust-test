#### program step

cargo build --release 
cargo objcopy --features stm32f103 --release -- -O ihex out.hex