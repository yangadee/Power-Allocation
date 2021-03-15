###########################################################################
## Makefile generated for component 'Test_Actuator_Mod_V4'. 
## 
## Makefile     : Test_Actuator_Mod_V4_rtw.mk
## Generated on : Sun Mar 14 05:18:02 2021
## Final product: .\Test_Actuator_Mod_V4.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPILER_COMMAND_FILE   Compiler command listing model reference header paths
# CMD_FILE                Command file
# MODELLIB                Static library target

PRODUCT_NAME              = Test_Actuator_Mod_V4
MAKEFILE                  = Test_Actuator_Mod_V4_rtw.mk
MATLAB_ROOT               = C:\PROGRA~1\MATLAB\R2021a
MATLAB_BIN                = C:\PROGRA~1\MATLAB\R2021a\bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)\win64
START_DIR                 = D:\Dropbox\1.���j�Ӥh����\10.�פ弶�g�Ѧ�\Thesis Matlab Code\Eason_program_ade\codegen\lib\Test_Actuator_Mod_V4
TGT_FCN_LIB               = ISO_C++11
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
COMPILER_COMMAND_FILE     = Test_Actuator_Mod_V4_rtw_comp.rsp
CMD_FILE                  = Test_Actuator_Mod_V4_rtw.rsp
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 
NODEBUG                   = 1
MODELLIB                  = Test_Actuator_Mod_V4.lib

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Microsoft Visual C++ 2019 v16.0 | nmake (64-bit Windows)
# Supported Version(s):    16.0
# ToolchainInfo Version:   2021a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# NODEBUG
# cvarsdll
# cvarsmt
# conlibsmt
# ldebug
# conflags
# cflags

#-----------
# MACROS
#-----------

MW_EXTERNLIB_DIR    = $(MATLAB_ROOT)\extern\lib\win64\microsoft
MW_LIB_DIR          = $(MATLAB_ROOT)\lib\win64
CPU                 = AMD64
APPVER              = 5.02
CVARSFLAG           = $(cvarsmt)
CFLAGS_ADDITIONAL   = -D_CRT_SECURE_NO_WARNINGS
CPPFLAGS_ADDITIONAL = -EHs -D_CRT_SECURE_NO_WARNINGS /wd4251
LIBS_TOOLCHAIN      = $(conlibs)

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: Microsoft Visual C Compiler
CC = cl

# Linker: Microsoft Visual C Linker
LD = link

# C++ Compiler: Microsoft Visual C++ Compiler
CPP = cl

# C++ Linker: Microsoft Visual C++ Linker
CPP_LD = link

# Archiver: Microsoft Visual C/C++ Archiver
AR = lib

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)\mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: NMAKE Utility
MAKE = nmake


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -Zi
C_OUTPUT_FLAG       = -Fo
LDDEBUG             = /DEBUG
OUTPUT_FLAG         = -out:
CPPDEBUG            = -Zi
CPP_OUTPUT_FLAG     = -Fo
CPPLDDEBUG          = /DEBUG
OUTPUT_FLAG         = -out:
ARDEBUG             =
STATICLIB_OUTPUT_FLAG = -out:
MEX_DEBUG           = -g
RM                  = @del
ECHO                = @echo
MV                  = @ren
RUN                 = @cmd /C

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = /nologo
CFLAGS               = $(cflags) $(CVARSFLAG) $(CFLAGS_ADDITIONAL) \
                       /O2 /Oy-
CPPFLAGS             = /TP $(cflags) $(CVARSFLAG) $(CPPFLAGS_ADDITIONAL) \
                       /O2 /Oy-
CPP_LDFLAGS          = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN)
CPP_SHAREDLIB_LDFLAGS  = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN) \
                         -dll -def:$(DEF_FILE)
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN)
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = $(ldebug) $(conflags) $(LIBS_TOOLCHAIN) \
                       -dll -def:$(DEF_FILE)



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = .\Test_Actuator_Mod_V4.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = 

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=Test_Actuator_Mod_V4

DEFINES = $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)\Test_Actuator_Mod_V4_data.cpp $(START_DIR)\rt_nonfinite.cpp $(START_DIR)\rtGetNaN.cpp $(START_DIR)\rtGetInf.cpp $(START_DIR)\Test_Actuator_Mod_V4_initialize.cpp $(START_DIR)\Test_Actuator_Mod_V4_terminate.cpp $(START_DIR)\Test_Actuator_Mod_V4.cpp $(START_DIR)\fileManager.cpp $(START_DIR)\eye.cpp $(START_DIR)\mtimes.cpp $(START_DIR)\svd.cpp $(START_DIR)\xnrm2.cpp $(START_DIR)\qpkwik.cpp $(START_DIR)\norm.cpp $(START_DIR)\minOrMax.cpp $(START_DIR)\xrotg.cpp $(START_DIR)\xgeqrf.cpp $(START_DIR)\xgerc.cpp $(START_DIR)\xorgqr.cpp $(START_DIR)\Actuator_Class_Mod_V4.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = Test_Actuator_Mod_V4_data.obj rt_nonfinite.obj rtGetNaN.obj rtGetInf.obj Test_Actuator_Mod_V4_initialize.obj Test_Actuator_Mod_V4_terminate.obj Test_Actuator_Mod_V4.obj fileManager.obj eye.obj mtimes.obj svd.obj xnrm2.obj qpkwik.obj norm.obj minOrMax.obj xrotg.obj xgeqrf.obj xgerc.obj xorgqr.obj Actuator_Class_Mod_V4.obj

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_ = /source-charset:utf-8
CFLAGS_BASIC = $(DEFINES) @$(COMPILER_COMMAND_FILE)

CFLAGS = $(CFLAGS) $(CFLAGS_) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_ = /source-charset:utf-8
CPPFLAGS_BASIC = $(DEFINES) @$(COMPILER_COMMAND_FILE)

CPPFLAGS = $(CPPFLAGS) $(CPPFLAGS_) $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################


!include $(MATLAB_ROOT)\rtw\c\tools\vcdefs.mak


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute set_environment_variables


all : build
	@cmd /C "@echo ### Successfully generated all binary outputs."


build : set_environment_variables prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


set_environment_variables : 
	@set INCLUDE=$(INCLUDES);$(INCLUDE)
	@set LIB=$(LIB)


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@cmd /C "@echo ### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS) -out:$(PRODUCT) @$(CMD_FILE)
	@cmd /C "@echo ### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

.c.obj :
	$(CC) $(CFLAGS) -Fo"$@" "$<"


.cpp.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.c.obj :
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(RELATIVE_PATH_TO_ANCHOR)}.cpp.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.c.obj :
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{$(START_DIR)}.cpp.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


{D:\Dropbox\1.���j�Ӥh����\10.�פ弶�g�Ѧ�\Thesis Matlab Code\Eason_program_ade}.c.obj :
	$(CC) $(CFLAGS) -Fo"$@" "$<"


{D:\Dropbox\1.���j�Ӥh����\10.�פ弶�g�Ѧ�\Thesis Matlab Code\Eason_program_ade}.cpp.obj :
	$(CPP) $(CPPFLAGS) -Fo"$@" "$<"


Test_Actuator_Mod_V4_data.obj : $(START_DIR)\Test_Actuator_Mod_V4_data.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\Test_Actuator_Mod_V4_data.cpp


rt_nonfinite.obj : $(START_DIR)\rt_nonfinite.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\rt_nonfinite.cpp


rtGetNaN.obj : $(START_DIR)\rtGetNaN.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\rtGetNaN.cpp


rtGetInf.obj : $(START_DIR)\rtGetInf.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\rtGetInf.cpp


Test_Actuator_Mod_V4_initialize.obj : $(START_DIR)\Test_Actuator_Mod_V4_initialize.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\Test_Actuator_Mod_V4_initialize.cpp


Test_Actuator_Mod_V4_terminate.obj : $(START_DIR)\Test_Actuator_Mod_V4_terminate.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\Test_Actuator_Mod_V4_terminate.cpp


Test_Actuator_Mod_V4.obj : $(START_DIR)\Test_Actuator_Mod_V4.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\Test_Actuator_Mod_V4.cpp


fileManager.obj : $(START_DIR)\fileManager.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\fileManager.cpp


eye.obj : $(START_DIR)\eye.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\eye.cpp


mtimes.obj : $(START_DIR)\mtimes.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\mtimes.cpp


svd.obj : $(START_DIR)\svd.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\svd.cpp


xnrm2.obj : $(START_DIR)\xnrm2.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\xnrm2.cpp


qpkwik.obj : $(START_DIR)\qpkwik.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\qpkwik.cpp


norm.obj : $(START_DIR)\norm.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\norm.cpp


minOrMax.obj : $(START_DIR)\minOrMax.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\minOrMax.cpp


xrotg.obj : $(START_DIR)\xrotg.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\xrotg.cpp


xgeqrf.obj : $(START_DIR)\xgeqrf.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\xgeqrf.cpp


xgerc.obj : $(START_DIR)\xgerc.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\xgerc.cpp


xorgqr.obj : $(START_DIR)\xorgqr.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\xorgqr.cpp


Actuator_Class_Mod_V4.obj : $(START_DIR)\Actuator_Class_Mod_V4.cpp
	$(CPP) $(CPPFLAGS) -Fo"$@" $(START_DIR)\Actuator_Class_Mod_V4.cpp


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(COMPILER_COMMAND_FILE) $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@cmd /C "@echo ### PRODUCT = $(PRODUCT)"
	@cmd /C "@echo ### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@cmd /C "@echo ### BUILD_TYPE = $(BUILD_TYPE)"
	@cmd /C "@echo ### INCLUDES = $(INCLUDES)"
	@cmd /C "@echo ### DEFINES = $(DEFINES)"
	@cmd /C "@echo ### ALL_SRCS = $(ALL_SRCS)"
	@cmd /C "@echo ### ALL_OBJS = $(ALL_OBJS)"
	@cmd /C "@echo ### LIBS = $(LIBS)"
	@cmd /C "@echo ### MODELREF_LIBS = $(MODELREF_LIBS)"
	@cmd /C "@echo ### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@cmd /C "@echo ### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@cmd /C "@echo ### CFLAGS = $(CFLAGS)"
	@cmd /C "@echo ### LDFLAGS = $(LDFLAGS)"
	@cmd /C "@echo ### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@cmd /C "@echo ### CPPFLAGS = $(CPPFLAGS)"
	@cmd /C "@echo ### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@cmd /C "@echo ### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@cmd /C "@echo ### ARFLAGS = $(ARFLAGS)"
	@cmd /C "@echo ### MEX_CFLAGS = $(MEX_CFLAGS)"
	@cmd /C "@echo ### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@cmd /C "@echo ### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@cmd /C "@echo ### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@cmd /C "@echo ### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@cmd /C "@echo ### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@cmd /C "@echo ### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	@if exist $(PRODUCT) $(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


