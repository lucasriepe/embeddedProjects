SET(SOURCES screens/ui_startup.c
    screens/ui_home.c
    screens/ui_secondScreen.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c)

add_library(ui ${SOURCES})
