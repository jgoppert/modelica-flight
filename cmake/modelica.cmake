macro(add_modelica NAME SRC FILES LIBS)
    set(${NAME}_src
        ${NAME}_01exo.c
        ${NAME}_02nls.c
        ${NAME}_03lsy.c
        ${NAME}_04set.c
        ${NAME}_05evt.c
        ${NAME}_06inz.c
        ${NAME}_07dly.c
        ${NAME}_08bnd.c
        ${NAME}_09alg.c
        ${NAME}_10asr.c
        ${NAME}_11mix.c
        ${NAME}_12jac.c
        ${NAME}_13opt.c
        ${NAME}_14lnz.c
        ${NAME}.c
        ${NAME}_functions.c
        ${NAME}_records.c)

    set(${NAME}_hdr
        ${NAME}_11mix.h
        ${NAME}_12jac.h
        ${NAME}_13opt.h
        ${NAME}_functions.h
        ${NAME}_includes.h
        ${NAME}_literals.h
        ${NAME}_model.h)

    set(${NAME}_data
        ${NAME}_info.json
        ${NAME}_init.xml)

    add_executable(${NAME}
        ${${NAME}_src}
        ${${NAME}_hdr})

    add_custom_command(OUTPUT ${${NAME}_src} ${${NAME}_hdr} ${${NAME}_data}
        COMMAND ${OMC} -q -s ${SRC} ${FILES} ${LIBS}
        DEPENDS ${SRC} ${FILES})

endmacro()

