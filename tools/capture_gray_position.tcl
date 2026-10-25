# Read-only raw-frame capture for the verified T312 ELF layout.
# Source after init (examine-end hook disabled), then call:
# capture_gray_position <samples 1..1500> <period_ms 10..100>
# Never resets, halts, flashes or requests motor output. Output goes to log.
proc capture_gray_position {samples period_ms} {
    if {$samples < 1 || $samples > 1500 || $period_ms < 10 || $period_ms > 100} {
        error "Invalid capture limits"
    }
    if {[lindex [read_memory 0x20000088 32 1] 0] != 0x54333132} {
        error "Capture layout requires T312"
    }
    echo "GRAY_POSITION_HEADER,index,tick_ms,frame_ms,samples,raw_status,timeouts,errors,unexpected,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8"
    set accepted 0
    for {set i 0} {$i < $samples} {incr i} {
        set d [read_memory 0x200090a8 8 2]
        set state [lindex $d 0]
        if {($state != 0 && $state != 5) || [lindex $d 1] != 0} {
            error "Controller must remain idle or DONE without faults"
        }
        foreach duty [read_memory 0x200090b6 16 2] {
            if {$duty != 0} {error "Motor command active"}
        }
        foreach ccr [read_memory 0x40000434 32 4] {
            if {$ccr != 0} {error "Motor output active"}
        }
        if {([lindex [read_memory 0x40010808 32 1] 0] & 1) != 0} {
            error "USER pressed; capture stopped"
        }
        set before [read_memory 0x20009238 32 9]
        set values [read_memory 0x200092a4 16 8]
        set confirm [read_memory 0x200092a4 16 8]
        set after [read_memory 0x20009238 32 9]
        set tick [lindex [read_memory 0x20009a78 32 1] 0]
        # Reject an in-flight update, invalid receiver, or an old frame.
        if {$before eq $after && $values eq $confirm &&
            (([lindex $after 0] >> 16) & 0xff) != 0 &&
            (($tick - [lindex $after 4]) & 0xffffffff) < 100} {
            set raw [expr {([lindex $after 0] >> 24) & 0xff}]
            set row [concat [list $i $tick [lindex $after 4] [lindex $after 5] $raw] [lrange $after 6 8] $values]
            echo "GRAY_POSITION,[join $row ,]"
            incr accepted
        }
        sleep $period_ms
    }
    echo "GRAY_POSITION_ACCEPTED=$accepted"
}
