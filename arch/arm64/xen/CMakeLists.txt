# Needed to separate definitions in common Xen headers
zephyr_compile_options($<$<COMPILE_LANGUAGE:ASM>:-D__ASSEMBLY__>)

#set_source_files_properties(hypercall.S PROPERTIES COMPILE_FLAGS -D__ASSEMBLY__)

zephyr_library_sources(hypercall.S)
zephyr_library_sources(enlighten.c)