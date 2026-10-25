# T320 only: no reset/halt, no motor request. Keep battery power after the run.
# After OpenOCD init with examine-end disabled:
# set capture_prefix build-local/captures/t320-run-01
# source tools/read_t320.tcl
if {![info exists capture_prefix]} {error "Set a unique capture_prefix first"}
if {[file exists ${capture_prefix}-ram.bin]} {error "Capture exists; choose another prefix"}
set g [read_memory 0x20000084 32 5]
if {[lindex $g 1] != 0x54333230} {error "Wrong firmware"}
if {[lindex $g 2] != 4 && [lindex $g 2] != 5} {error "No finished/aborted run"}
set d [read_memory 0x200090a8 8 128]
if {[lindex $d 0] != 0 && [lindex $d 0] != 5 && [lindex $d 0] != 6} {error "Controller active"}
foreach offset {14 15 16 17} {if {[lindex $d $offset] != 0} {error "PWM active"}}
foreach ccr [read_memory 0x40000434 32 4] {if {$ccr != 0} {error "CCR active"}}
if {([lindex [read_memory 0x40010808 32 1] 0] & 1) != 0} {error "USER pressed"}
set m [read_memory 0x20009a84 32 9]
if {[lindex $m 0] != 0x53504431 || [lindex $m 1] != 28 || [lindex $m 2] != 104} {error "Wrong speed record ABI"}
file mkdir [file dirname $capture_prefix]
dump_image ${capture_prefix}-ram.bin 0x20000000 49152
dump_image ${capture_prefix}-speed-meta.bin 0x20009a84 36
dump_image ${capture_prefix}-speed-samples.bin 0x20009aa8 5824
dump_image ${capture_prefix}-metadata.bin 0x20000084 20
dump_image ${capture_prefix}-diag.bin 0x200090a8 128
dump_image ${capture_prefix}-controller.bin 0x20009128 204
dump_image ${capture_prefix}-trace.bin 0x200000a8 36864
dump_image ${capture_prefix}-arc-debug.bin 0x200092d4 928
dump_image ${capture_prefix}-run-metrics.bin 0x20000098 16
echo "GATE=$g"
echo "SPEED_META=$m"
echo "OLED=[read_memory 0x20009674 32 3]"
echo "TICK=[read_memory 0x2000b178 32 1]"
