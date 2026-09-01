# Bert NCR/Z5380 SCSI driver

`Drv-SCSIBert.asm` is the SymbOS MSX mass-storage driver for Bert
Hoogenboom's NCR5380/Z5380 SCSI cartridge.

## Hardware selection

The released hardware uses I/O ports `30h` through `37h`; this is the default
selected by `SCSIBASE`. A board, CPLD and cartridge-ROM build which decodes
`D0h` through `D7h` can be supported by changing only `SCSIBASE` to `#D0`.
The driver, CPLD and cartridge ROM must all use the same range.

The three-byte `SCSISLT` field is required by the SymbOS MSX driver ABI. The
driver does not use it because the controller is I/O-mapped. It is unrelated
to the SD card slot selection used by SD Mapper drivers.

The SMD3 type and driver-ID metadata both identify this as storage type `3`
(SCSI). Storage type `2` is SD. Keeping the Bert driver as type `3` prevents
SymSetup from asking the user to choose SD card slot 1 or 2.

## Disks and targets

- SCSI target ID 0 uses Bert's custom partition table, as required by the
  MSX-DOS cartridge ROM.
- Other target IDs use a standard MBR partition table.
- The target ID is taken from the channel bits in the SymbOS device record.
- Transfers use SCSI `READ(10)` and `WRITE(10)` commands with 32-bit LBAs.

## Build and verification

Assemble `Drv-SCSIBert1.asm` in the normal SymbOS MSX source-tree layout. The
wrapper expects `SymbOS-File-Const.asm` one directory above the driver source
and writes `-SCBERT.DRV` to the standard MSX output directory.

The `30h` build was verified with 86 relocation entries in the 1983 emulator:
SymbOS 4.0 booted with 512 KB RAM, read its system files through the SCSI
driver, and loaded the configured desktop background. A matching `30h`
cartridge ROM was used for the MSX-DOS boot stage.
