# Read-only STM32F103 C10B encoder trace for a manual push.
# Source after OpenOCD init, then call:
# capture_encoder <uwTick> <encoder> <app_diagnostics> <samples> <period_ms>
# Resolve the three RAM addresses from the currently flashed ELF before use.
# Does not reset, halt, start motors, or alter firmware/registers.
# Stop the capture on a nonzero motor command, a nonzero CCR, an unlocked
# calibration flag, a fault, or a non-stopped app state.
# Reads are sequential, not atomic: compare raw/software counts at stationary
# endpoints; a difference during motion can be normal sampling latency.
proc capture_encoder {tick_addr encoder_addr app_addr samples period_ms} {
    if {$samples < 1 || $samples > 1200 || $period_ms < 20 || $period_ms > 100} {
        error "capture requires 1..1200 samples and 20..100 ms period"
    }
    echo "ENCODER_HEADER,index,tick_ms,raw_a,raw_b,a_lo,a_hi,b_lo,b_hi,previous_packed,last_update_ms"
    for {set i 0} {$i < $samples} {incr i} {
        set status [lindex [read_memory $app_addr 32 1] 0]
        if {($status & 0xff00ffff) != 0} {
            error "app is not stopped, fault-free and calibration-locked"
        }
        foreach duty [read_memory [expr {$app_addr + 14}] 16 2] {
            if {$duty != 0} {error "nonzero motor command; trace stopped"}
        }
        foreach ccr [read_memory 0x40000434 32 4] {
            if {$ccr != 0} {error "nonzero motor CCR; trace stopped"}
        }
        set tick [lindex [read_memory $tick_addr 32 1] 0]
        set raw_a [lindex [read_memory 0x40000824 32 1] 0]
        set raw_b [lindex [read_memory 0x40013424 32 1] 0]
        set software [read_memory $encoder_addr 32 6]
        set row [concat [list $i $tick $raw_a $raw_b] $software]
        echo "ENCODER_TRACE,[join $row ,]"
        if {$i + 1 < $samples} {sleep $period_ms}
    }
}
