#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=
FINAL_IMAGE=${DISTDIR}/Practica05.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=
FINAL_IMAGE=${DISTDIR}/Practica05.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=Imagenes.s Main.s Util.s Interrupciones.s Configuracion.s Inicio.s

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/Imagenes.o ${OBJECTDIR}/Main.o ${OBJECTDIR}/Util.o ${OBJECTDIR}/Interrupciones.o ${OBJECTDIR}/Configuracion.o ${OBJECTDIR}/Inicio.o
POSSIBLE_DEPFILES=${OBJECTDIR}/Imagenes.o.d ${OBJECTDIR}/Main.o.d ${OBJECTDIR}/Util.o.d ${OBJECTDIR}/Interrupciones.o.d ${OBJECTDIR}/Configuracion.o.d ${OBJECTDIR}/Inicio.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/Imagenes.o ${OBJECTDIR}/Main.o ${OBJECTDIR}/Util.o ${OBJECTDIR}/Interrupciones.o ${OBJECTDIR}/Configuracion.o ${OBJECTDIR}/Inicio.o

# Source Files
SOURCEFILES=Imagenes.s Main.s Util.s Interrupciones.s Configuracion.s Inicio.s



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/Practica05.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=30F4013
MP_LINKER_FILE_OPTION=,--script="p30F4013.gld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/Imagenes.o: Imagenes.s  .generated_files/flags/default/9f419df7a5a4bc64c8ab9cf10e73ea30e1d99da9 .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Imagenes.o.d 
	@${RM} ${OBJECTDIR}/Imagenes.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Imagenes.s  -o ${OBJECTDIR}/Imagenes.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Imagenes.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Main.o: Main.s  .generated_files/flags/default/a61062728c14239a11bc8e226837a452fd6c19af .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Main.o.d 
	@${RM} ${OBJECTDIR}/Main.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Main.s  -o ${OBJECTDIR}/Main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Main.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Util.o: Util.s  .generated_files/flags/default/f414e54e0b34ebabfef791a0bb7a1df41e7398c6 .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Util.o.d 
	@${RM} ${OBJECTDIR}/Util.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Util.s  -o ${OBJECTDIR}/Util.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Util.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Interrupciones.o: Interrupciones.s  .generated_files/flags/default/776724e49c4520366dd29f918a5cf447e1129b6e .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Interrupciones.o.d 
	@${RM} ${OBJECTDIR}/Interrupciones.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Interrupciones.s  -o ${OBJECTDIR}/Interrupciones.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Interrupciones.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Configuracion.o: Configuracion.s  .generated_files/flags/default/97e34e8a113f50cb87976bc746baaed67c750e29 .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Configuracion.o.d 
	@${RM} ${OBJECTDIR}/Configuracion.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Configuracion.s  -o ${OBJECTDIR}/Configuracion.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Configuracion.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Inicio.o: Inicio.s  .generated_files/flags/default/bd967db064cdcec4b0846f3b945e2971e1624990 .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Inicio.o.d 
	@${RM} ${OBJECTDIR}/Inicio.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Inicio.s  -o ${OBJECTDIR}/Inicio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Inicio.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/Imagenes.o: Imagenes.s  .generated_files/flags/default/cad7078ef9bfc05026ba392170c6237f1f9ac04e .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Imagenes.o.d 
	@${RM} ${OBJECTDIR}/Imagenes.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Imagenes.s  -o ${OBJECTDIR}/Imagenes.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Imagenes.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Main.o: Main.s  .generated_files/flags/default/6f771f2c364dc4ddb3113e006a09f00f1d395ba6 .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Main.o.d 
	@${RM} ${OBJECTDIR}/Main.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Main.s  -o ${OBJECTDIR}/Main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Main.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Util.o: Util.s  .generated_files/flags/default/e0591f40153846659eea952e17f91f828c3cd833 .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Util.o.d 
	@${RM} ${OBJECTDIR}/Util.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Util.s  -o ${OBJECTDIR}/Util.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Util.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Interrupciones.o: Interrupciones.s  .generated_files/flags/default/9b7f657dd9b06bf24302141be2c50c5cb2fa8dac .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Interrupciones.o.d 
	@${RM} ${OBJECTDIR}/Interrupciones.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Interrupciones.s  -o ${OBJECTDIR}/Interrupciones.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Interrupciones.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Configuracion.o: Configuracion.s  .generated_files/flags/default/756d521e5be8fbef129da7ca7b37078fab419d28 .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Configuracion.o.d 
	@${RM} ${OBJECTDIR}/Configuracion.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Configuracion.s  -o ${OBJECTDIR}/Configuracion.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Configuracion.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/Inicio.o: Inicio.s  .generated_files/flags/default/a847836b59af6f9e40176fa9e37bef45d45c44ac .generated_files/flags/default/78b5023c5e9ef3d435e434410e50b575e7db83ae
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Inicio.o.d 
	@${RM} ${OBJECTDIR}/Inicio.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  Inicio.s  -o ${OBJECTDIR}/Inicio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)    -Wa,-MD,"${OBJECTDIR}/Inicio.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp="${DFP_DIR}/xc16"
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/Practica05.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    p30F4013.gld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/Practica05.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)      -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	
else
${DISTDIR}/Practica05.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   p30F4013.gld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/Practica05.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	${MP_CC_DIR}/xc16-bin2hex ${DISTDIR}/Practica05.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   -mdfp="${DFP_DIR}/xc16" 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
