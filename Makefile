obj-y  = AddressMapping.o ClockDomain.o Inline_ECC.o MemorySystemTop.o Transaction.o TimingCalculate.o
obj-y += BankState.o IniReader.o Rank.o
obj-y += parameter/

# 1. names
EXE_NAME=DRAMSim
STATIC_LIB_NAME := libdramsim.a
LIB_NAME=libdramsim.so
LIB_NAME_A=libdramsim.a
LIB_NAME_MACOS=libdramsim.dylib

SRC := $(wildcard *.cpp)
PROTO ?= HBM

TIMING_ALL := $(wildcard TSC_*cpp)
TIMING_SEL := TSC_$(PROTO).cpp

TIMING_OTHERS := $(filter-out $(TIMING_SEL),$(TIMING_ALL))
SRC := $(filter-out $(TIMING_ALL),$(SRC))
SRC := $(filter-out MemoryController.cpp,$(SRC))

OBJ = $(addsuffix .o, $(basename $(SRC)))
INI_SRC = $(wildcard ./parameter/$(PROTO)/*.ini)
INI_OBJ = $(patsubst %.ini,%.o,$(INI_SRC))
EMBED_S = ./embed_asm.S

LIB_SRC := $(filter-out TraceBasedSim.cpp,$(SRC))
LIB_OBJ := $(addsuffix .o, $(basename $(LIB_SRC)))

#build portable objects (i.e. with -fPIC)
POBJ = $(addsuffix .po, $(basename $(LIB_SRC)))

REBUILDABLES=$(OBJ) ${POBJ} $(EXE_NAME) $(LIB_NAME) $(STATIC_LIB_NAME)


# 2. compiler flags
CXXFLAGS=-DNO_STORAGE -Wall -DDEBUG_BUILD -fPIC -std=c++14 -g -O0
OPTFLAGS=${SOC_CFLAG}

ifdef DEBUG
ifeq ($(DEBUG), 1)
OPTFLAGS= -O0 -g
endif
endif
CXXFLAGS+=$(OPTFLAGS)
CXXFLAGS+= -Wfatal-errors

ifeq ($(PROTO),LP)
    CXXFLAGS+= -DUSE_TSC_LP
else ifeq ($(PROTO),DDR)
    CXXFLAGS+= -DUSE_TSC_DDR
else ifeq ($(PROTO),HBM)
    CXXFLAGS+= -DUSE_TSC_HBM
endif
# =====================================================================
# 3. 跨平台命令适配 (Cross-Platform Commands)
# =====================================================================
ifeq ($(OS),Windows_NT)
    # Windows 环境下的 CMD 命令
    RM      = del /Q /F
    RMDIR   = rmdir /S /Q
    # 注意：这里必须用 = 而不能用 :=，确保宏参数 $(1) 延迟求值
    MKDIR   = if not exist $(1) mkdir $(subst /,\,$(1))
    MKDIR_P = if not exist $(1) mkdir $(subst /,\,$(1))
    DEVNULL = nul
    CLEAN_CMD = $(RM) *.o *.po $(EXE_NAME) $(EXE_NAME).exe $(LIB_NAME) $(LIB_NAME_A) $(STATIC_LIB_NAME) run.log leak.log qcachegrind.log perf.data 2>nul || cd .
else
    # Linux/macOS 环境下的 Bash 命令
    RM      = rm -f
    RMDIR   = rm -rf
    MKDIR   = mkdir
    MKDIR_P = mkdir -p
    DEVNULL = /dev/null
    CLEAN_CMD = $(RM) $(OBJ) $(POBJ) $(INI_OBJ) $(EXE_NAME) $(LIB_NAME) $(LIB_NAME_A) $(STATIC_LIB_NAME) run.log leak.log qcachegrind.log perf.data
endif
# =====================================================================

DEPDIR := .d
$(shell $(call MKDIR_P, $(DEPDIR)) >$(DEVNULL) 2>&1)

DEPDIR := .d
$(shell $(call MKDIR_P, $(DEPDIR)) >/dev/null 2>&1)
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td
BUILDSTATICLIB := ar -rcs

CC          := g++ -O3
CXX         := g++ -O3
BB          := ar

COMPILE.c = $(CC) $(DEPFLAGS) $(CFLAGS) $(CPPFLAGS)  $(GPROF_OPT) $(INCPATH) -c
COMPILE.cc = $(CXX) $(DEPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(GPROF_OPT) $(INCPATH) -c
POSTCOMPILE = @echo Done $@

%.o : %.cpp
%.o : %.cpp $(DEPDIR)/%.d
	$(COMPILE.cc) $(OUTPUT_OPTION) $<
	$(POSTCOMPILE)

# 8. Default
default:
	@echo "=========================================================================================="
	@echo " COMMAND                        | DESCRIPTION"
	@echo "================================+========================================================="
	@echo " make ini_obg or 0              | Compile ini.o For Linxi"
	@echo " make compile or 1              | Compile For LPDDR UT"
	@echo " make sim or 2                  | Run the simulation with output log print to terminal"
	@echo " make sim_log or 3              | Run the simulation with output log print to run.log"
	@echo " make compile_so or 4           | Compile Dynamic Library File For ESL System"
	@echo " make compile_a or 5            | Compile Static Library File For ESL System"
	@echo " make clean or 6                | Not delete DRAMSim"
	@echo " make clean_all or 7            | Delete the temp file"
	@echo " make all or 8                  | run compile -> sim_log"
	@echo " make leak or 9                 | run leak -> leak.log"
	@echo " make qcachegrind or 10         | run qcachegrind -> qcachegrind.log"
	@echo " make qcachegrind_log or 11     | run qcachegrind qcachegrind.log"
	@echo " make callgrind_control or 12   | update qcachegrind.log"
	@echo " make perf_stat or 13           | run perf stat"
	@echo " make perf_record or 14         | run perf record"
	@echo " make perf_report or 15         | run perf report"
	@echo "=========================================================================================="

ini_obg 0: ${INI_OBJ}

$(INI_OBJ): %.o: %.ini
	g++ -DFILE='"$<"' -c -o $@ $(EMBED_S)

compile 1: ${LIB_NAME} $(EXE_NAME) 
	@$(call MKDIR, log)

#   $@ target name, $^ target deps, $< matched pattern
$(EXE_NAME): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ 
	@echo "Built $@ successfully" 

$(LIB_NAME): $(OBJ)
	g++ -shared -Wl,-soname,$@ -o $@ $^
	@echo "Built $@ successfully"

compile_so 4: $(OBJ)
	g++ -shared -Wl,-soname,$(LIB_NAME) -o $(LIB_NAME) AddressMapping.o BankState.o ClockDomain.o IniReader.o Inline_ECC.o MemorySystemTop.o Rank.o Transaction.o TimingCalculate.o
	@echo "Built $(LIB_NAME) successfully"

compile_a 5: $(OBJ)
	ar -rc $(LIB_NAME_A) AddressMapping.o BankState.o ClockDomain.o IniReader.o Inline_ECC.o MemorySystemTop.o Rank.o Transaction.o TimingCalculate.o Rmw.o

$(STATIC_LIB_NAME): $(LIB_OBJ)
	$(AR) crs $@ $^

$(LIB_NAME_MACOS): $(POBJ)
	g++ -dynamiclib -o $@ $^
	@echo "Built $@ successfully"

clean 6:
	$(CLEAN_CMD)
	-@$(RMDIR) log 2>nul || cd .
	-@$(RMDIR) .d 2>nul || cd .

clean_all 7:
	$(CLEAN_CMD)
	-@$(RMDIR) log 2>nul || cd .
	-@$(RMDIR) .d 2>nul || cd .

# 9. Auto dependency generation 
$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d
-include $(wildcard $(patsubst %,$(DEPDIR)/%.d,$(basename $(SRC))))

# 10. UT sim
sim 2:
	@$(call MKDIR, log)
ifeq ($(OS),Windows_NT)
	.\$(EXE_NAME)
else
	./$(EXE_NAME)
endif

# 10. UT sim with run.log
sim_log 3:
	@$(call MKDIR, log)
ifeq ($(OS),Windows_NT)
	.\$(EXE_NAME) > run.log 2>&1
else
	./$(EXE_NAME) > run.log 2>&1
endif

all 8: compile sim_log

leak 9:
	@$(call MKDIR_P, log)
	@valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --error-limit=no --verbose --log-file=leak.log ./DRAMSim

qcachegrind 10:
	@valgrind --tool=callgrind --trace-children=yes --callgrind-out-file=./qcachegrind.log ./DRAMSim

qcachegrind_log 11:
	@~/tools/qcachegrind qcachegrind.log

callgrind_control 12:
	@callgrind_control -d

perf_stat 13:
	@perf stat ./DRAMSim

perf_record 14:
	@perf record -F 999 ./DRAMSim

perf_report 15:
	@perf report
