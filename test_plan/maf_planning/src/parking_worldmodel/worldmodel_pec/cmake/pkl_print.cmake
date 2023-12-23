macro(PKL_PRINT msg)
    string(ASCII 27 Esc)
    set(COLOR_RED ${Esc}[31m)
    set(COLOR_RESET ${Esc}[m)
    message("${COLOR_RED} ${msg} ${COLOR_RESET}")


endmacro()
