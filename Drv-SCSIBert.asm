;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;@                                                                            @
;@            S y m b O S   -   M S X   D e v i c e   D r i v e r             @
;@                         NCR5380 SCSI Interface                             @
;@                                                                            @
;@             (c) by Bert Hoogenboom / SymbiosiS (Jorn Mika)                 @
;@                                                                            @
;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

;corrected version (v1.2), based on bertSCSI-write-read10.asm - changes
;(no colons in comments - the WinApe assembler treats them as statement
;separators even inside comments)
;- SCSIRW banking fixed - BNKMONX is now called with HL=target address, the
;  corrected address and the returned page-0 block value are actually used
;  (the original passed a stale HL, threw both results away and transferred
;  to the raw caller address -> garbage on any transfer into another 64K bank,
;  i.e. every larger file load)
;- interrupt discipline fixed - the transfer block is only mapped at #0000
;  during the SCSI data phase, with DI held for the whole mapped stretch;
;  every 512 bytes a short interrupt window is opened (page 0 restored,
;  EI, pending IRQ runs, DI, block remapped) - the original kept the mapping
;  active with interrupts enabled and only wrapped DI/EI around single bytes
;- relocation fixed - "relocate_count equ 0", DRVSTART/DRVEND and the raw
;  "save" are gone; assemble with the WinApe assembler or a recent sjasmplus
;  via the standard wrapper (Drv-SCSIBert1.asm), which also provides the
;  SymbOS constants (the private copies were removed)
;- the 4th jump table entry no longer points at the MOF slot inside the
;  16-byte prologue (which the boot loader separates from the code); it now
;  points to SCSICLO in the code area
;- the SCSI target ID now comes from the device record (stodatsub bit 4-7,
;  set by the config app), no longer from the SymbOS device slot number;
;  the Bert-table/MBR decision in SCSIACT is based on it as well
;- error codes - hardware selftest failure returns stoerrmis, "unit not
;  ready" returns stoerrrdy, a stalled data phase returns stoerrabo
;- the selection timeout loop yields to the scheduler via RST #30
;- data-out phase got the same stall timeout as data-in
;- dead code removed (CHKBSY/RESCMD, error message text)
;- v1.3 size pass (~110 bytes) - the priority-mask computation at the start
;  of SCSIARBIT was dead code (its result in C was never read, the
;  arbitration-lost check it belonged to is gone anyway) and got removed
;  together with the _OWNID/_OWNIDB variables (the own ID is the constant
;  SCSIOWNB now); shared subroutines SCSICLR/SCSIMAP/SCSICNT/SCSIRST/
;  SCSIPARGET replace duplicated blocks; INP/OUT share their prologue;
;  SCSIACT uses the OS variable stoadry instead of an own device-number copy
;v1.2 was verified on real hardware (2026-08-29); the v1.3 size pass is
;untested again - re-test reading AND writing after assembling.
;- v1.4 (2026-08-29) - SCSIPARBERT bugfix - a Bert-table partition reached
;  through a type-5 chain entry was returning the wrong start sector (in
;  practice, whatever was near the start of the disk, i.e. partition 1's
;  data) because the entry's start field was used as-is instead of being
;  added to the sector of the table it was found in. Confirmed against the
;  real MSX-DOS driver (MSXDOS DRIVER\DRIVES.SUB, DRIV_3/DRIV_4/DRIV_A),
;  which does that same 32bit add. Only affects SCSI ID 0 (Bert table);
;  SCSIPARMBR (any other ID) was already correct and is untouched.
;- v1.4 metadata correction - SMD3 storage type 3 is SCSI. Type 4 is not a
;  valid SymbOS MSX 4.0 storage type. SymSetup 4.0 reuses its SD-card-slot
;  wording for the type-3 channel selection; that text is not part of this
;  driver. The three-byte SCSISLT field remains because it is part of the
;  driver ABI, but this I/O-mapped driver never reads it.

;This driver reads the on-disk partition table in one of two formats,
;chosen by SCSI target ID -
; - ID 0 uses Bert's own custom table (4 entries per sector at #1C0/#1D0/
;   #1E0/#1F0, a type-5 "extended" entry chains to another table sector).
;   This is required for the MSX-DOS boot disk, since MSX-DOS's own ROM
;   only understands this format.
; - any other ID uses the standard SymbOS/MBR partition table (same layout
;   as the IDE/SD/USB drivers), for a second, SymbOS-only disk.
;These two formats cannot coexist on the same disk (their entries overlap
;the same bytes), so the choice is made purely by which SCSI ID is being
;accessed, not by inspecting the disk content.
;
;NOTE - "Partition table BERT interface.txt"'s field table and its worked
;pseudocode example disagree by 2 bytes on where the start/length fields
;sit; this driver follows the field table (entry+#02/#06/#0A) as
;authoritative.
;
;I/O base port - SCSIBASE below is set to #30 for the current hardware model.
;If you are building for the other Bert SCSI board revision, change that one
;equ to its I/O base - everything else (SCSIP0-SCSIP7) is derived from it.

;--- MAIN ROUTINES ------------------------------------------------------------
;### SCSIOUT -> write sectors (512b)
;### SCSIINP -> read sectors (512b)
;### SCSIACT -> read and init media (only hardware- and partiondata, no filesystem)
;### SCSICLO -> close device (no-op)

;--- WORK ROUTINES ------------------------------------------------------------
;### SCSIUNITRDY -> test unit ready (only hardware)
;### SCSITARGET  -> calculate target ID bit
;### SCSIGO      -> stage-independent tail - select target, run command, check status
;### SCSIARBIT   -> do SCSI arbitration, selection and the full phase state machine
;### SCSICDB     -> build a 10 byte command block
;### SCSICHN     -> read the SCSI target ID for a device from its data record
;### SCSISEC     -> add the partition offset to a logical sector number
;### SCSIWIN     -> between-sector interrupt window during the data phase
;### SCSIRAWRD   -> read one absolute sector (no partition offset) into (stobufx)
;### SCSIPARBERT -> locate the Nth partition in Bert's table (SCSI ID 0)
;### SCSIPARMBR  -> locate the Nth partition in a standard MBR (any other ID)
;### SCSIPARGET  -> fetch the start LBA of a located partition entry
;### SCSICLR     -> clear the staged 10 byte command block
;### SCSIMAP     -> map the transfer block, init sector counter/stall budget
;### SCSICNT     -> per-byte counting, opens SCSIWIN every 512 bytes
;### SCSIRST     -> reset the SCSI controller registers
;### CHKSCSI     -> check SCSI interface hardware


;==============================================================================
;### HEADER ###################################################################
;==============================================================================

org #1000-32
relocate_start

db "SMD3"               ;ID (generic SymbOS mass-storage driver header)
dw scsiend-scsijmp       ;code length
dw relocate_count        ;number of relocate table entries
ds 8                     ;*reserved*
db 4,1,3                 ;Version Minor, Version Major, Type (0=FDC, 1=IDE, 2=SD, 3=SCSI)
db "Bert SCSI    "       ;comment (13 chars)

scsijmp dw SCSIINP,SCSIOUT,SCSIACT,SCSICLO
SCSIMOF ret:dw 0         ;50Hz slot - nothing to do for SCSI (no motor)
        db 32*3+13      ;bit[0-4]=driver ID (13=Bert SCSI), bit[5-7]=storage type (3=SCSI)
        ds 4
SCSISLT   DS 3            ;MSX slot configuration of the interface, written by
                          ;the boot loader. Bert SCSI is I/O-port mapped, not
                          ;memory-mapped, so the driver never reads it - it
                          ;only has to exist at this fixed offset (start of the
                          ;relocated code area).

;*** driver-global state (saved between accesses) ***
_TRGID    DS 1           ;current target ID, bit-encoded
_TRGDEV   DS 1            ;current target ID, raw (0-7), read from the device
                          ;record (stodatsub bit 4-7) by SCSICHN
_SCSICNT  DS 1            ;sector count for the command in progress
_SCSISTS  DS 1            ;SCSI status byte captured during the last status phase
_SCSIERR  DS 1            ;sticky "illegal phase" error flag, cleared per command
SCSIBLK   DS 1            ;page-0 block value returned by BNKMONX - used to map
                          ;the transfer block during the data phase and to remap
                          ;it after each interrupt window
_CMDB0    DS 1
_CMDB1    DS 1
_CMDB2    DS 1
_CMDB3    DS 1
_CMDB4    DS 1
_CMDB5    DS 1
_CMDB6    DS 1
_CMDB7    DS 1
_CMDB8    DS 1
_CMDB9    DS 1

SCSIACTA  DS 2            ;device data record address, temp storage across SCSIACT
SCSIACTPN DS 1            ;requested partition number, temp storage across SCSIACT

;*** Bert-format partition scan state (used only for SCSI ID 0) ***
SCSIPARN  DS 1            ;partitions left to skip (1=this is the one we want)
SCSIPARCS DS 4            ;chain target sector for the next table (32bit)
SCSIPARCF DS 1            ;1 = a type-5 (extended) entry was found in the current table
SCSIPARHP DS 1            ;chain-hop guard, prevents an infinite loop on a circular table


;*** Variables and Constants ***

;status/error codes and device record offsets (stotyp.../stoerr.../stodat...)
;come from SymbOS-File-Const.asm, included by the build wrapper Drv-SCSIBert1.asm

stobnkx equ #815A       ;memory mapping, when low level routines read/write sector data
bnkmonx equ #8112       ;set special memory mapping during mass storage access
bnkmofx equ #8115       ;reset special memory mapping during mass storage access
bnkdofx equ #8136       ;hide mass storage device rom
stoadrx equ #8157       ;get device data record
clcd16x equ #8166       ;HL=BC/DE, DE=BC mod DE
stobufx equ #815B       ;address of 512byte buffer
stoadry equ #8169       ;current device

;*** Bert SCSI I/O ports (NCR5380) ***
SCSIBASE equ #30        ;I/O base port of the interface. The two known Bert SCSI
                         ;board revisions may use different bases - if this is
                         ;the "other" model, change only this equ.
SCSIP0   equ SCSIBASE+0
SCSIP1   equ SCSIBASE+1
SCSIP2   equ SCSIBASE+2
SCSIP3   equ SCSIBASE+3
SCSIP4   equ SCSIBASE+4
SCSIP5   equ SCSIBASE+5
SCSIP6   equ SCSIBASE+6
SCSIP7   equ SCSIBASE+7

SCSIOWNB equ %10000000  ;host adapter's own SCSI ID (fixed at 7), bit-encoded
                         ;(1 shl ID) - this is what belongs on the SCSI data
                         ;bus during arbitration and selection, never the raw
                         ;ID number; adjust if the interface is jumpered to a
                         ;different ID

_SELPOLL equ #0600      ;inner tight-poll count of the selection wait (~20ms)
_SELTRY  equ 13          ;outer selection retries, each followed by an RST #30
                         ;yield (~1/50s) -> total selection timeout ~250ms+

_ARBTMO equ #FFFF       ;bound for the ARBIT1 (bus-free wait) and ARBIT2
                         ;(arbitration-in-progress wait) loops, and for the
                         ;data-phase per-byte stall check - none of these had a
                         ;timeout originally and could hang forever if the bus
                         ;got stuck. Not calibrated to a real time value - it's
                         ;the largest 16bit count on purpose, to turn "hangs
                         ;forever" into "eventually reports an error". These
                         ;waits normally exit within microseconds, so they stay
                         ;tight polls (no RST #30).

;*** Bert's own partition table (see "Partition table BERT interface.txt") -
;*** used for SCSI ID 0, the MSX-DOS-bootable disk ***
bertparadr equ #1c0        ;first entry inside the sector
bertparsz  equ #10          ;size of one entry
bertpartyp equ #02          ;system byte (1=DOS 12bit FAT, 5=extended/chain)
bertparbeg equ #06          ;4 byte LE start sector / next table sector

;*** standard SymbOS MBR partition table (same layout as the IDE/SD/USB
;*** drivers) - used for any SCSI ID other than 0 ***
mbrparadr equ #1be         ;start of partition table inside the MBR
mbrpartyp equ #04           ;system/type byte offset within an entry
mbrparbeg equ #08           ;4 byte LE start-sector offset within an entry

mbrpartok  db #01,#11, #04,#06,#0e,#14,#16,#1e, #0b,#0c,#1b,#1c
mbrpartan  equ 12


;==============================================================================
;### MAIN ROUTINES ############################################################
;==============================================================================

;### SCSIOUT -> write sectors (512b)
;### Input      A=device (0-7), IY,IX=logical sector number, B=number of sectors,
;###            DE=source address, (stobnkx)=banking config
;### Output     CF=0 -> ok
;###            CF=1 -> A=error code
;### Destroyed  AF,BC,DE,HL,IX,IY
SCSIOUT: LD C,#2A         ;WRITE(10) - full 32bit LBA, needed for disks over ~1GB
         JR SCSIIO

;### SCSIINP -> read sectors (512b)
;### Input      A=device (0-7), IY,IX=logical sector number, B=number of sectors,
;###            DE=destination address, (stobnkx)=banking config
;### Output     CF=0 -> ok
;###            CF=1 -> A=error code
;### Destroyed  AF,BC,DE,HL,IX,IY
SCSIINP: LD C,#28         ;READ(10) - see the note by WRITE(10) above in SCSIOUT

;### SCSIIO -> shared prologue of SCSIINP/SCSIOUT
;### Input      C=opcode, rest as above
SCSIIO:  PUSH AF
         LD A,B
         LD (_SCSICNT),A
         POP AF
         CALL SCSISEC     ;IY,IX=absolute LBA, (_TRGDEV)=target ID (keeps C)
         LD A,C

;### SCSIRW -> shared tail of SCSIINP/SCSIOUT/SCSIRAWRD
;### Input      A=opcode, IY,IX=absolute LBA, DE=buffer address,
;###            (_SCSICNT)=sector count, (_TRGDEV)=target ID, (stobnkx)=banking
SCSIRW:  CALL SCSICDB
         EX DE,HL          ;HL=buffer address (BNKMONX expects it in HL)
         LD A,(stobnkx)
         DI
         CALL bnkmonx      ;HL=corrected buffer address, A=page-0 block value
         LD (SCSIBLK),A    ;remember the block - the data phase maps it and
                           ;remaps it after every interrupt window
         CALL bnkmofx      ;page 0 back to the OS until data actually flows
         EI
         LD A,(_TRGDEV)
         JP SCSIGO

;### SCSIACT -> read and init media (only hardware- and partiondata, no filesystem)
;### Input      A=device (0-7). The SCSI target ID is taken from the device
;###            record (stodatsub bit 4-7, set by the config app). Target ID 0
;###            is read using Bert's custom partition table (required for the
;###            MSX-DOS boot disk); any other target ID is read using the
;###            standard SymbOS/MBR partition table.
;### Output     CF=0 -> ok
;###            CF=1 -> A=error code
;### Destroyed  AF,BC,DE,HL,IX,IY
SCSIACT: CALL CHKSCSI
         RET C                    ;hardware self test failed (A=stoerrmis)

         LD A,(stoadry)           ;current device number, stored there by STOACT
         CALL SCSICHN              ;(_TRGDEV)=target ID, HL=device data record
         LD (SCSIACTA),HL
         LD DE,stodatsub
         ADD HL,DE
         LD A,(HL)
         AND #F                    ;A=requested partition number (0=whole disk)
         LD (SCSIACTPN),A

         CALL SCSIUNITRDY
         JR NC,SCSIACT2
         CP stoerrsec
         JR NZ,SCSIACT1
         LD A,stoerrrdy            ;target answered but is not ready (no medium)
SCSIACT1: SCF
         RET

SCSIACT2: LD A,(_TRGDEV)
         OR A
         JR NZ,SCSIACTM            ;non-zero target ID -> standard MBR table

         LD A,(SCSIACTPN)
         CALL SCSIPARBERT          ;target ID 0 -> Bert's custom table
         JR SCSIACTP

SCSIACTM: LD IY,0                  ;read sector 0 (MBR) into the OS scratch buffer
         LD IX,0
         CALL SCSIRAWRD
         RET C
         LD A,(SCSIACTPN)
         CALL SCSIPARMBR

SCSIACTP: RET C

         LD HL,(SCSIACTA)          ;*** store result in the device data record
         LD (HL),stotypoky         ;device ready
         INC HL
         LD (HL),stomedhdi         ;media type (reusing the generic "hard disc" type)
         LD DE,stodatbeg-stodattyp
         ADD HL,DE
         LD (HL),C:INC HL          ;store start sector (32bit, little endian)
         LD (HL),B:INC HL
         PUSH IY
         POP DE
         LD (HL),E:INC HL
         LD (HL),D
         XOR A
         RET

;### SCSICLO -> close device (no-op, this driver needs no cleanup on removal).
;###            Lives in the code area on purpose - the jump table must never
;###            point into the 16-byte prologue, which the boot loader stores
;###            separately from the code.
SCSICLO: RET


;==============================================================================
;### WORK ROUTINES ############################################################
;==============================================================================

;### SCSIUNITRDY -> test unit ready (only hardware)
;### Input      (_TRGDEV)=target ID
;### Output     CF=0 -> ok
;###            CF=1 -> A=error code
;### Destroyed  AF,BC,DE,HL
SCSIUNITRDY:
        CALL SCSICLR    ;all-zero CDB = TEST UNIT READY (6-byte command, the
                         ;target only requests _CMDB0-5)
        LD A,(_TRGDEV)
        JP SCSIGO

;### SCSITARGET -> calculate target ID bit
;### Input      A = SCSI ID of target drive (0-7)
;### Output     A = SCSI ID of target drive, bit format
;### Destroyed  AF
SCSITARGET: PUSH BC
        AND #07         ;Bit 0-2 are the SCSI ID of the drive
        LD B,A
        LD A,1
        JR Z,TARGE2
TARGE1: RLA
        DJNZ TARGE1
TARGE2: LD (_TRGID),A
        POP BC
        RET

;### SCSIGO -> select the target, run the staged command, check completion status
;### Input      A=target ID (0-7), (_CMDB0-9)=staged command block,
;###            HL=data buffer address (only relevant for commands with a data
;###            phase; must be the BNKMONX-corrected address, with (SCSIBLK)
;###            holding the matching page-0 block value)
;### Output     CF=0 -> ok
;###            CF=1 -> A=error code
;### Destroyed  AF,BC,DE,HL
SCSIGO: CALL SCSITARGET
        CALL SCSIARBIT
        RET C                    ;no device responded / bus error
        LD A,(_SCSISTS)
        OR A
        RET Z                    ;SCSI status GOOD -> success
        LD A,stoerrsec
        SCF
        RET

;### SCSIARBIT -> do SCSI arbitration and selection, then run the full phase
;###              state machine (command/data/status/message) for the staged command.
;###              Runs with the normal OS memory mapping and interrupts enabled;
;###              only the data phase maps the transfer block at #0000 (with DI
;###              held while it is mapped, see SCSIDI/SCSIDOUT).
;### Input       _TRGID, _CMDB0-_CMDB9, HL=data buffer address
;### Output      Carry set if no device responded
;### Destroyed   AF,BC,DE,HL,IX
SCSIARBIT:
        XOR A
        LD (_SCSIERR),A
        PUSH BC
ARBIT1: LD DE,_ARBTMO    ;bus-free wait, bounded
        IN A,(SCSIP4)   ;read SCSI bus Status
        OR A
        JR NZ,ARBIT1W
        IN A,(SCSIP0)   ;read SCSI data
        OR A
        JR NZ,ARBIT1W
        IN A,(SCSIP4)   ;read SCSI bus Status
        OR A
        JR Z,ARBIT1OK
ARBIT1W: DEC DE
        LD A,D
        OR E
        JR NZ,ARBIT1
        JR ARBITNF               ;timed out waiting for the bus to go free
ARBIT1OK:
        LD A,SCSIOWNB    ;arbitration asserts exactly your own single ID bit
        OUT (SCSIP0),A  ;on the data bus
        LD A,%00000001
        OUT (SCSIP2),A  ;Write SCSI mode
        LD DE,_ARBTMO
ARBIT2: IN A,(SCSIP1)   ;read Initiator command Register
        BIT 6,A                 ;check arbitration in progress
        JR NZ,ARBIT8
        IN A,(SCSIP4)   ;read SCSI bus Status
        AND %11111110
        JR NZ,ARBITL
        DEC DE
        LD A,D
        OR E
        JR NZ,ARBIT2
        JR ARBITNF               ;timed out waiting for arbitration to complete
ARBIT8: NOP
        NOP                     ;arbitration settle delay (~2.2us)
                                 ;no arbitration-lost check here - this host is
                                 ;hardcoded to SCSI ID 7, the highest possible
                                 ;priority, so it can never actually lose
        JR ARBITS
ARBITL: XOR A
        OUT (SCSIP2),A  ;reset mode register
        OUT (SCSIP1),A  ;reset initiator command
        OUT (SCSIP2),A  ;reset mode again
        LD A,32         ;fixed retry delay (the R register must not be used -
                         ;the kernel scheduler relies on it for timing)
ARBTL1: DEC A
        JR NZ,ARBTL1
        JR ARBIT1
ARBITS: LD A,%00001100
        OUT (SCSIP1),A  ;set SEL and busy
        NOP                     ;wait 1.2 usec.
        LD A,(_TRGID)    ;selection asserts the initiator's and target's ID
        OR SCSIOWNB      ;bits together (bit-encoded), not their raw numbers
        OUT (SCSIP0),A  ;Send Own ID and target ID
        LD A,%00001101
        OUT (SCSIP1),A
        XOR A
        OUT (SCSIP2),A
        OUT (SCSIP4),A
        LD A,%00000101  ;reset BSY
        OUT (SCSIP1),A
        LD B,_SELTRY    ;selection timeout - _SELTRY slices of a short tight
ARBIT3A: LD DE,_SELPOLL  ;poll, an RST #30 yield (~1/50s, registers preserved)
                         ;after each slice - keeps the system alive while a
                         ;missing target runs into the timeout
ARBIT3: IN A,(SCSIP4)
        BIT 6,A
        JR NZ,ARBIT7
        DEC DE
        LD A,D
        OR E
        JR NZ,ARBIT3
        RST #30                  ;yield - normal OS mapping is active here
        DJNZ ARBIT3A
ARBITNF: CALL SCSIRST    ;shared "give up, report target not found" cleanup -
        LD A,stoerrdvp   ;also used by the ARBIT1/ARBIT2 timeouts above
        SCF
        JR ARBIT6       ;target not found
ARBIT4: CALL SCSIRST    ;reset controller after use
ARBIT5: LD A,(_SCSIERR)
        OR A
        JR Z,ARBIT6
        LD A,stoerrsec
        SCF
ARBIT6: POP BC
        RET

ARBIT7: LD A,%00000000
        OUT (SCSIP1),A  ;target selected, continue with information transfer
        LD A,%00100010  ;set DMA mode and parity check
        OUT (SCSIP2),A
        POP BC

ARBITE: PUSH BC
        LD C,SCSIP0
        LD DE,_CMDB0            ;DE=command block pointer (becomes HL during the
                                 ;command phase); HL already holds the caller's
                                 ;data buffer address (becomes HL again during the
                                 ;data phase) - see SCSICO below.
SCSI1:  XOR A
        OUT (SCSIP2),A  ;RESET DMA
        OUT (SCSIP1),A  ;STOP ASSERT DATA BUS
SCSI10: IN A,(SCSIP4)
        RLCA
        JR C,ARBIT5     ;RESET
        RLCA
        JR NC,ARBIT4    ;BSY
        RLCA
        JR NC,SCSI10    ;REQ
        RLCA
        JP C,SCSIMS     ;MSG
        RLCA
        JP C,SCSICM     ;C/D (JP, not JR - the enlarged data phases put SCSICM
                        ;out of JR range)
        RLCA
        JR C,SCSIDI     ;I/O

;SCSI DATA OUT
;maps the transfer block at #0000 (DI!) while bytes are streaming; opens an
;interrupt window after every 512 bytes; unmaps again on phase change or stall
SCSIDOUT:
        OUT (SCSIP3),A
        LD A,#82
        OUT (SCSIP2),A
        OUT (SCSIP5),A
        LD A,01
        OUT (SCSIP1),A  ;ASSERT DATA BUS
        CALL SCSIMAP    ;DI, map transfer block, init sector counter/stall budget
SCSIO1: IN A,(SCSIP5)
        BIT 6,A
        JR Z,SCSIO2
        OUTI            ;output from (HL) to (C), dec (B), inc(HL)
        LD A,%00000001  ;reset ack bit
        OUT (SCSIP1),A
        CALL SCSICNT    ;count byte, open the interrupt window every 512 bytes
        JR SCSIO1
SCSIO2: IN A,(SCSIP4)
        AND #DE
        CP #40
        JR NZ,SCSIO3    ;phase changed - leave the data phase
        DEC DE
        LD A,D
        OR E
        JR NZ,SCSIO1
        JP ARBITAB      ;timed out mid-transfer waiting for the target
SCSIO3: JR SCSI5        ;restore page 0, back to the phase dispatcher

;SCSI DATA IN
;maps the transfer block at #0000 (DI!) while bytes are streaming; opens an
;interrupt window after every 512 bytes; unmaps again on phase change or stall
SCSIDI: LD C,SCSIP6
        OUT (SCSIP3),A
        LD A,#82
        OUT (SCSIP2),A  ;set dma mode
        OUT (SCSIP7),A  ;start dma mode
        XOR A
        OUT (SCSIP1),A  ;reset ack bit
        CALL SCSIMAP    ;DI, map transfer block, init sector counter/stall budget
SCSII1: IN A,(SCSIP5)
        BIT 6,A         ;dma request
        JR Z,SCSII2
        INI             ;input from (C) to (HL), dec (B), inc(HL)
        IN A,(SCSIP0)
        XOR A           ;reset ack bit
        OUT (SCSIP1),A
        CALL SCSICNT    ;count byte, open the interrupt window every 512 bytes
        JR SCSII1
SCSII2: IN A,(SCSIP4)   ;no dma req
        AND #DE
        CP #44          ;i/o en bsy?
        JR NZ,SCSI5     ;phase changed - handle normally, not a stall
        DEC DE
        LD A,D
        OR E
        JR NZ,SCSII1
        JP ARBITAB      ;timed out mid-transfer waiting for the target
SCSI5:  CALL bnkmofx    ;page 0 back to the OS
        EI
        LD C,SCSIP0
        JP SCSI1

;shared data-phase abort - unmap, reset the controller, report stoerrabo.
;stack level here matches ARBIT6 (the SCSIARBIT-entry push was reloaded at
;ARBIT7/ARBITE), so the shared exit can be reused
ARBITAB: CALL bnkmofx   ;page 0 back to the OS
        EI
        CALL SCSIRST
        LD A,stoerrabo
        SCF
        JP ARBIT6

;### SCSIMAP -> map the transfer block at #0000 (DI!) and init the 512-byte
;###            sector counter (IXH/IXL) and the per-byte stall budget (DE)
;### Destroyed  AF,DE
SCSIMAP: DI
        LD A,(SCSIBLK)
        OUT (#fc),A     ;map the transfer block at #0000
        DB #DD:LD H,2   ;IXH/IXL = 512-byte sector counter (2 x 256)
        DB #DD:LD L,0
        LD DE,_ARBTMO   ;bounded wait per byte, reset on every byte that
                         ;actually transfers, so only genuine stalling counts
        RET

;### SCSICNT -> count one transferred byte, reset the stall budget; after 512
;###            bytes restart the counter and fall into the interrupt window
;### Destroyed  AF,DE
SCSICNT: LD DE,_ARBTMO   ;progress happened - reset the stall budget
        DB #DD:DEC L
        RET NZ
        DB #DD:DEC H
        RET NZ
        DB #DD:LD H,2   ;sector complete - restart the counter, open the window

;### SCSIWIN -> between-sector interrupt window during the data phase -
;###            restores page 0, lets one pending interrupt run, then remaps
;###            the transfer block. The SCSI handshake simply waits meanwhile.
;### Destroyed  AF
SCSIWIN: CALL bnkmofx    ;page 0 back to the OS
        EI
        NOP             ;a pending interrupt is accepted here
        DI
        LD A,(SCSIBLK)
        OUT (#fc),A     ;transfer block back at #0000
        RET

;### SCSIRST -> reset the SCSI controller registers
;### Destroyed  AF
SCSIRST: XOR A
        OUT (SCSIP0),A
        OUT (SCSIP1),A
        OUT (SCSIP2),A
        OUT (SCSIP3),A
        RET

;SCSI MESSAGE
SCSICM: RLCA
        JR C,SCSICI

;SCSI COMMAND PHASE
SCSICO: OUT (SCSIP3),A
        EX DE,HL
        LD A,01
        OUT (SCSIP1),A  ;ASSERT DATA BUS
SCSIC1: IN A,(SCSIP4)
        AND #FE
        CP %01101000
        JR NZ,SCSIC2
        OUTI            ;output from (HL) to (C), dec (B), inc(HL)
        LD A,%00010001  ;reset ack bit
        OUT (SCSIP1),A
SCSIC3: IN A,(SCSIP4)
        BIT 5,A
        JR NZ,SCSIC3
        LD A,%00000001
        OUT (SCSIP1),A
        JR SCSIC1
SCSIC2: CP %01001000
        JR Z,SCSIC1
        EX DE,HL
SCSI2:  JP SCSI1

;SCSI STATUS PHASE
SCSICI: OUT (SCSIP3),A
        LD A,#82
        OUT (SCSIP2),A  ;SET DMA MODE
        OUT (SCSIP7),A  ;START DMA RECEIVE
        IN A,(SCSIP6)
        LD (_SCSISTS),A
        IN A,(C)
        JR SCSI2

;SCSI MESSAGE PHASE
SCSIMS: RLCA
        JR NC,PHAERR    ;C/D
        RLCA
        JR C,SCSIMI     ;I/O

;MESSAGE OUT PHASE
SCSIMO: OUT (SCSIP3),A
        LD A,01
        OUT (SCSIP1),A  ;ASSERT DATA BUS
        LD A,#82
        OUT (SCSIP2),A  ;SET DMA MODE
        OUT (SCSIP5),A  ;START DMA SEND
        XOR A           ;message out is always 0 - no IDENTIFY/negotiation is
                         ;sent, and target disconnect/reselection is not
                         ;implemented
        OUT (C),A
        JR SCSI2

;MESSAGE IN PHASE
SCSIMI: OUT (SCSIP3),A
        LD A,#82
        OUT (SCSIP2),A  ;SET DMA MODE
        OUT (SCSIP7),A  ;START DMA RECEIVE
        IN A,(SCSIP6)
        IN A,(C)
        JP SCSI1

PHAERR:                 ;mark the command as failed and let the bus settle back to idle
        LD A,1
        LD (_SCSIERR),A
        LD A,#80        ;a message without C/D asserted doesn't exist
        OUT (SCSIP1),A
        XOR A
        OUT (SCSIP1),A
        JR SCSI2

;### SCSICDB -> stage a 10 byte SCSI command block (READ(10)/WRITE(10)
;###            format). Full 32bit LBA, needed since the target disk is
;###            larger than the ~1GB a 21-bit (READ(6)/WRITE(6)) LBA can address.
;### Input      A=opcode, IY,IX=32bit LBA, (_SCSICNT)=sector count (max 255 -
;###            SCSIINP/SCSIOUT only ever pass a single byte count in B)
;### Output     _CMDB0-9 filled in
;### Destroyed  AF,B,HL
SCSICDB: PUSH AF
         CALL SCSICLR           ;bytes 1,6,7,9 (LUN, group/reserved, transfer
                                 ;length MSB, control) stay 0 from the clear
         POP AF
         LD (_CMDB0),A
         LD A,IYH
         LD (_CMDB2),A          ;LBA bits 31-24
         LD A,IYL
         LD (_CMDB3),A          ;LBA bits 23-16
         LD A,IXH
         LD (_CMDB4),A          ;LBA bits 15-8
         LD A,IXL
         LD (_CMDB5),A          ;LBA bits 7-0
         LD A,(_SCSICNT)
         LD (_CMDB8),A          ;transfer length LSB
         RET

;### SCSICLR -> clear the staged 10 byte command block
;### Destroyed  AF,B,HL
SCSICLR: LD HL,_CMDB0
         LD B,10
         XOR A
SCSICLR1: LD (HL),A
         INC HL
         DJNZ SCSICLR1
         RET

;### CHKSCSI -> check SCSI interface hardware
;### Output     CF=0 -> ok, CF=1 -> A=stoerrmis (hardware not found)
;### Destroyed  AF
CHKSCSI: IN A,(SCSIP4)
        INC A
        JR Z,CHKSC1     ;INTERFACE ERROR
        LD A,%01000000
        OUT (SCSIP1),A
        IN A,(SCSIP4)
        INC A
        JR NZ,CHKSC1    ;INTERFACE ERROR
        OUT (SCSIP1),A
        OR A
        RET             ;SCSI Hardware is OK

CHKSC1: XOR A           ;A=0=stoerrmis (hardware not found)
        OUT (SCSIP1),A
        SCF             ;set carry to indicate hardware not working
        RET


;==============================================================================
;### SUB ROUTINES #############################################################
;==============================================================================

;### SCSICHN -> read the SCSI target ID for a device from its data record
;###            (stodatsub bit 4-7 = channel, set by the config application)
;### Input      A=device (0-7)
;### Output     (_TRGDEV)=target ID (0-7), HL=device data record
;### Destroyed  AF,BC
SCSICHN: CALL stoadrx           ;HL=device data record
         PUSH HL
         LD BC,stodatsub
         ADD HL,BC
         LD A,(HL)
         RRCA
         RRCA
         RRCA
         RRCA
         AND #07                 ;channel -> SCSI target ID (0-7)
         LD (_TRGDEV),A
         POP HL
         RET

;### SCSISEC -> adds the partition offset to a logical sector number and
;###            fetches the SCSI target ID of the device
;### Input      A=device (0-7), IY,IX=logical sector number
;### Output     IY,IX=absolute LBA (32bit), (_TRGDEV)=target ID
;### Destroyed  AF,F,HL
SCSISEC: PUSH BC
         CALL SCSICHN           ;HL=device data record, (_TRGDEV) set
         LD BC,stodatbeg
         ADD HL,BC
         LD C,(HL):INC HL
         LD B,(HL):INC HL
         ADD IX,BC
         LD C,(HL):INC HL
         LD B,(HL)
         JR NC,SCSISEC1
         INC BC
SCSISEC1: ADD IY,BC
         POP BC
         RET

;### SCSIRAWRD -> read one ABSOLUTE sector (no partition offset applied) into (stobufx)
;### Input      IY,IX=absolute sector number, (_TRGDEV)=target ID (set by SCSICHN)
;### Output     CF=0 -> ok, data in (stobufx); CF=1 -> A=error code
;### Destroyed  AF,BC,DE,HL,IX,IY
SCSIRAWRD: LD A,1
         LD (_SCSICNT),A
         LD DE,(stobufx)
         LD A,#28         ;READ(10) - kept in sync with SCSIINP/SCSIOUT above
         JP SCSIRW

;### SCSIPARBERT -> locate the Nth partition in Bert's custom table (SCSI ID 0
;###                only), following type-5 (extended) entries as chain links
;###                to further table sectors
;### Input      A=partition number (0=whole disk), (_TRGDEV)=target ID
;### Output     CF=0 -> BC=start LBA low16, IY=start LBA high16
;###            CF=1 -> A=error code (stoerrpno)
;### Destroyed  AF,BC,DE,HL,IX,IY
SCSIPARBERT: OR A
         JP Z,SCSIPARW           ;0 -> whole disk, start=0
         LD (SCSIPARN),A
         XOR A
         LD HL,0
         LD (SCSIPARCS+0),HL
         LD (SCSIPARCS+2),HL
         LD A,64
         LD (SCSIPARHP),A        ;bound the number of chained tables we will follow

SCSIPARBL: LD A,(SCSIPARHP)
         DEC A
         LD (SCSIPARHP),A
         JR Z,SCSIPARBER          ;too many hops -> assume a corrupt/circular table
         XOR A
         LD (SCSIPARCF),A
         LD IX,(SCSIPARCS+0)
         LD IY,(SCSIPARCS+2)
         CALL SCSIRAWRD            ;read this table sector into (stobufx)
         RET C
         LD IX,(stobufx)
         LD DE,bertparadr
         ADD IX,DE
         LD B,4
SCSIPARBE: LD A,(IX+bertpartyp)
         CP 1
         JR NZ,SCSIPARBE1
         LD A,(SCSIPARN)
         DEC A
         LD (SCSIPARN),A
         JR NZ,SCSIPARBE3
         LD BC,bertparbeg        ;*** this is the partition we want
         ADD IX,BC
         CALL SCSIPARGET          ;BC=entry's own start field (low16), IY=(high16)
         PUSH IY
         POP HL
         EX DE,HL                 ;DE=entry's own start field (high16)
         LD HL,(SCSIPARCS+0)      ;the start field is relative to the sector of
         ADD HL,BC                ;the table it was found in (0 for the top-level
         LD B,H                   ;table, an absolute chain-target sector for any
         LD C,L                   ;table reached via a type-5 entry) - add it in,
         LD HL,(SCSIPARCS+2)      ;matching the real MSX-DOS driver (DRIVES.SUB
         ADC HL,DE                ;DRIV_3/DRIV_4), which does the same 32bit add
         PUSH HL                  ;before using an entry's start field
         POP IY
         XOR A
         RET
SCSIPARBE1: CP 5
         JR NZ,SCSIPARBE3
         PUSH BC                 ;B holds the DJNZ entries-remaining counter below
         LD C,(IX+bertparbeg+0)  ;*** extended entry -> remember it as the next table
         LD B,(IX+bertparbeg+1)
         LD (SCSIPARCS+0),BC
         LD C,(IX+bertparbeg+2)
         LD B,(IX+bertparbeg+3)
         LD (SCSIPARCS+2),BC
         POP BC
         LD A,1
         LD (SCSIPARCF),A
SCSIPARBE3: LD DE,bertparsz
         ADD IX,DE
         DJNZ SCSIPARBE
         LD A,(SCSIPARCF)
         OR A
         JR NZ,SCSIPARBL           ;follow the chain to the next table
SCSIPARBER: LD A,stoerrpno          ;no (more) entries -> partition not found
         SCF
         RET

;### SCSIPARGET -> fetch the 32bit start LBA of a located partition entry
;### Input      IX=address of the 4 byte little endian start-sector field
;### Output     CF=0, BC=start LBA low16, IY=start LBA high16
;### Destroyed  A,BC,IY
SCSIPARGET: LD C,(IX+0)
         LD B,(IX+1)
         PUSH BC
         LD C,(IX+2)
         LD B,(IX+3)
         PUSH BC
         POP IY
         POP BC
         XOR A
         RET

;### SCSIPARMBR -> locate the Nth primary partition inside the MBR held in
;###               (stobufx) (any SCSI ID other than 0)
;### Input      A=partition number (0=whole disk)
;### Output     CF=0 -> BC=start LBA low16, IY=start LBA high16
;###            CF=1 -> A=error code (stoerrpno/stoerrptp)
;### Destroyed  F,HL,IX
SCSIPARMBR: OR A
         JR Z,SCSIPARW           ;0 -> whole disk, start=0
         LD IX,(stobufx)
         LD BC,mbrparadr
SCSIPARM1: ADD IX,BC
         LD BC,16
         DEC A
         JR NZ,SCSIPARM1
SCSIPARM2: LD A,(IX+mbrpartyp)
         LD B,A
         OR A
         LD A,stoerrpno          ;partition does not exist -> error
         SCF
         RET Z
         LD A,B
         LD HL,mbrpartok
         LD B,mbrpartan
SCSIPARM3: CP (HL)
         JR Z,SCSIPARM4
         INC HL
         DJNZ SCSIPARM3
         LD A,stoerrptp          ;partition type not supported -> error
         SCF
         RET
SCSIPARM4: LD BC,mbrparbeg
         ADD IX,BC
         JR SCSIPARGET
SCSIPARW: LD BC,0
         LD IY,0
         XOR A
         RET

scsiend

relocate_table
relocate_end
