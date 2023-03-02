idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src" "common" "nes" "nes/apu" "nes/cartridge" "nes/cartridge/mappers" "nes/cpu" "nes/generic/ram" "nes/generic/rom" "nes/joy" "nes/joy/controllers" "nes/ppu" "nes/wiring"
  PRIV_INCLUDE_DIRS "."
  REQUIRES box-emu-hal
  )
# target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error-format)
# target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-char-subscripts -Wno-attributes)
# target_compile_definitions(${COMPONENT_LIB} PUBLIC USE_NES_NOFRENDO)
  # SRC_DIRS "src" "nofrendo/cpu" "nofrendo/libsnss" "nofrendo/nes" "nofrendo/sndhrdw" "nofrendo/mappers" "nofrendo"
  # PRIV_INCLUDE_DIRS "nofrendo/cpu" "nofrendo/libsnss" "nofrendo/nes" "nofrendo/sndhrdw" "nofrendo"
