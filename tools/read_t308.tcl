# Current T308 RAM layout; run with OpenOCD init, no reset/halt before capture.
file mkdir build-local/captures
# T308 only. Read before reset or starting a new run.
set gate [read_memory 0x20000084 32 5]
if {[lindex $gate 1] != 0x54333038} {error "Wrong firmware"}
set phase [lindex $gate 2]
if {$phase != 4 && $phase != 5} {error "No completed run; do not reset"}
set d [read_memory 0x200090a8 8 128]
foreach offset {14 15 16 17} {if {[lindex $d $offset] != 0} {error "PWM not zero"}}
foreach ccr [read_memory 0x40000434 32 4] {if {$ccr != 0} {error "CCR not zero"}}
dump_image build-local/captures/t308-run-after.bin 0x200090a8 128
dump_image build-local/captures/t308-run-metadata.bin 0x20000084 20
dump_image build-local/captures/t308-run-trace.bin 0x200000a8 36864
dump_image build-local/captures/t308-run-controller.bin 0x20009128 188
echo "DUAL_GATE=$gate"
echo "DUAL_TICK=[read_memory 0x200096d8 32 1]"
sleep 1000
dump_image build-local/captures/t308-run-stopped.bin 0x200090a8 128
echo "FINAL_CCR=[read_memory 0x40000434 32 4]"
echo "OLED=[read_memory 0x200092c4 32 3]"
dump_image build-local/captures/t308-run-run-metrics.bin 0x20000098 16
echo "RUN_METRICS=[read_memory 0x20000098 32 4]"
