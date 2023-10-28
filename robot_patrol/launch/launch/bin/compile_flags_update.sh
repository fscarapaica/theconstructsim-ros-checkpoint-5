#! /bin/bash

# -----------------------------------------------------------------------------
# Update /home/compile_flags.txt file if new include folders are found in
# /home/user, and restart clangd.
#
# That file is generated automatically in the xterm container by a process
# also called ide_autocompleation.sh when the content of ~/catkin_ws/src changes
#
# clang uses /home/compile_commands.json, which is a link to
# ~/.ide_autocompletion/updated_compile_commands.json, which is a copy of
# ~/.ide_autocompletion/compile_commands.json
# -----------------------------------------------------------------------------

FILE_TEMPLATE=/home/compile_flags_template.txt
MOST_RECENT=/home/compile_flags_updated.txt
COMPILE_FLAGS=/home/compile_flags.txt
color_cyan='\e[36m'
color_reset='\033[0m'
date_format='[%Y-%m-%d %T]'
function log_cyan {
    echo -e "${color_cyan}$(date +"${date_format}") ${@} ${color_reset}";
}

function create_initial_compile_flags {
    cp -fv "${FILE_TEMPLATE}" "${COMPILE_FLAGS}"
}

function generate_updated_file {
    # ----------------------------------------------------
    # Generate an updated 'compile_flags_updated.txt' file with 'include'
    # folders found in /home/user
    # ----------------------------------------------------
    cp -fv "${FILE_TEMPLATE}" "${MOST_RECENT}"
    for folder in $(find /home/user -type d -name include); do
        echo "-I${folder}" | tee --append "${MOST_RECENT}" &>/dev/null
    done
}

function restart_clangd_if_needed {
    # -------------------------------------------------------------------------
    # Check whether compile_flags.txt is different from compile_flags_updated.txt
    # If so, update compile_flags.txt and restart clangd
    #   clangd is responsible for auto completion in the IDE
    # -------------------------------------------------------------------------
    has_changed=$(cmp --silent ${COMPILE_FLAGS} ${MOST_RECENT} || echo true)

    if [[ ${has_changed} = "true" ]]; then
            log_cyan 'Restarting clangd'
            cp -v ${MOST_RECENT} ${COMPILE_FLAGS}
            set -x; kill $(pidof clangd); set +x;
    else
        log_cyan 'No need to restart clangd'
    fi
}

function main {

    create_initial_compile_flags

    while true; do

        generate_updated_file
        restart_clangd_if_needed

        sleep 30
    done

}

main