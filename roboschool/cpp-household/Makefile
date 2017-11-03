UNAME := $(shell uname -s)
OBJDIRR=.build-release
OBJDIRD=.build-debug
OBJDIRR_PY=.build-release-py
OBJDIRD_PY=.build-debug-py

ifeq ($(UNAME),Linux)
  PKG  =pkg-config
  MOC  =moc -qt=5
  LIBS =-L/usr/lib64 -lm -lGL -lGLU
  INC  =-I/usr/include
  BOOST_MT=
  ifneq ($(USE_PYTHON3),0)
    HAVE_PYTHON3=$(shell which python3)
  endif
  ifneq ($(HAVE_PYTHON3),)
    PYTHON = $(shell python3 -c "import sys; print('%i.%i' % sys.version_info[:2])")
    ifneq (,$(findstring linuxbrew,$(HAVE_PYTHON3)))
      $(info Linux, python3, linuxbrew branch)
      BOOST_PYTHON3_POSTFIX = 3
    else ifneq (,$(findstring conda,$(HAVE_PYTHON3)))
      $(info Linux, python3, anaconda branch, will query Ubuntu python3 for boost_python)
      BOOST_PYTHON3_POSTFIX = -py$(shell /usr/bin/python3 -c "import sys; print('%i%i' % sys.version_info[:2])")
    else
      $(info Linux, python3, Ubuntu branch)
      BOOST_PYTHON3_POSTFIX = -py$(shell python3 -c "import sys; print('%i%i' % sys.version_info[:2])")
    endif
  else
    $(info Linux, python 2 branch)
    PYTHON = $(shell python -c "import sys; print('%i.%i' % sys.version_info[:2])")
  endif
  RPATH=-Wl,-z,origin -Wl,--disable-new-dtags -Wl,-rpath,
endif

ifeq ($(UNAME),Darwin)
  ifneq ("$(wildcard /usr/local/opt/qt5/bin/moc"), "")
    $(info Mac, assumed python3, homebrew branch)
    MOC =/usr/local/opt/qt5/bin/moc
  else
    $(info Mac, assumed python3, anaconda branch)
    MOC =moc
  endif
  PKG   =pkg-config
  LIBS  =-framework OpenGL
  INC   =-I/System/Library/Frameworks/OpenGL.framework/Headers
  BOOST_PYTHON3_POSTFIX = 3
  BOOST_MT=-mt
  PYTHON = $(shell python3 -c "import sys; print('%i.%i' % sys.version_info[:2])")
  RPATH =
endif

$(info Link against python $(PYTHON))
INC     += `$(PKG) --cflags Qt5Widgets Qt5OpenGL`
LIBS    += -lstdc++ $(RPATH) `$(PKG) --libs Qt5OpenGL Qt5Widgets`
INC     += $(INCLUDE_FLAGS)
LIBS    += $(LIB_FLAGS)
INC_PY     += `$(PKG) --cflags python-$(PYTHON)`
LIBS_PY    += `$(PKG) --libs python-$(PYTHON)`

ifeq ($(PYTHON),2.7)
    BOOST_PYTHON = -lboost_python
else
    BOOST_PYTHON = -lboost_python$(BOOST_PYTHON3_POSTFIX)
endif

CC=gcc-5
LINK=gcc-5
#AR=ar ru
AR=ar rcs
AR_OUT=
LINK_OUT= -o
MINUS_O = -o
CFLAGS   = -std=c++11 -Wall -Wno-unused-variable -Wno-unused-function -Wno-deprecated-register -fPIC -DBT_USE_DOUBLE_PRECISION -g -O3 -march=native $(INC)
CFLAGSD  = -std=c++11 -Wall -Wno-unused-variable -Wno-unused-function -Wno-deprecated-register -fPIC -DBT_USE_DOUBLE_PRECISION -g -DDEBUG $(INC)

SHARED  = -shared
DEPENDS = -MMD -MF $@.dep

EVERY_BIN=../robot-test-tool ../robot-test-tool_d ../cpp_household.so ../cpp_household_d.so ../libroboschool.so ../libroboschool_d.so

SIM = \
 physics-bullet.cpp \
 assets-mesh.cpp \
 random-world-tools.cpp \
 render-glwidget.cpp \
 render-hud.cpp \
 render-simple.cpp \
 render-simple-primitives.cpp

ifneq ("$(wildcard /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.0)", "")
$(info Hardware render (turn on shadows))
SIM     += render-ssao.cpp
CFLAGS  += -DUSE_SSAO
CFLAGSD += -DUSE_SSAO
else
$(info Slow hardware or software render (no shadows))
endif

CFLAGS_PY   = $(CFLAGS) $(INC_PY) -DBOOST_SMART_POINTER
CFLAGSD_PY  = $(CFLAGSD) $(INC_PY) -DBOOST_SMART_POINTER

TWND = \
 test-tool-qt4.cpp

PYTH = python-binding.cpp
CPP = roboschool.cpp

SIM_R = $(patsubst %.cpp, $(OBJDIRR)/%.o, $(SIM))
SIM_D = $(patsubst %.cpp, $(OBJDIRD)/%.o, $(SIM))
SIM_R_PY = $(patsubst %.cpp, $(OBJDIRR_PY)/%.o, $(SIM))
SIM_D_PY = $(patsubst %.cpp, $(OBJDIRD_PY)/%.o, $(SIM))
TWND_R = $(patsubst %.cpp, $(OBJDIRR)/%.o, $(TWND))
TWND_D = $(patsubst %.cpp, $(OBJDIRD)/%.o, $(TWND))
TWND_R_PY = $(patsubst %.cpp, $(OBJDIRR_PY)/%.o, $(TWND))
TWND_D_PY = $(patsubst %.cpp, $(OBJDIRD_PY)/%.o, $(TWND))
PYTH_R = $(patsubst %.cpp, $(OBJDIRR_PY)/%.o, $(PYTH))
PYTH_D = $(patsubst %.cpp, $(OBJDIRD_PY)/%.o, $(PYTH))
CPP_R = $(patsubst %.cpp, $(OBJDIRR)/%.o, $(CPP))
CPP_D = $(patsubst %.cpp, $(OBJDIRD)/%.o, $(CPP))

EVERY_OBJ_R_PY = $(SIM_R_PY) $(TWND_R_PY) $(PYTH_R)
EVERY_OBJ_D_PY = $(SIM_D_PY) $(TWND_D_PY) $(PYTH_D)
EVERY_OBJ_R = $(SIM_R) $(TWND_R) $(CPP_R)
EVERY_OBJ_D = $(SIM_D) $(TWND_D) $(CPP_D)
DEP = $(patsubst %.o,%.o.dep, $(EVERY_OBJ_R) $(EVERY_OBJ_D))
DEP_PY = $(patsubst %.o,%.o.dep, $(EVERY_OBJ_R_PY) $(EVERY_OBJ_D_PY))

all: dirs $(EVERY_BIN)

$(OBJDIRR)/test-tool-qt4.o: .generated/test-tool-qt4.moc
.generated/test-tool-qt4.moc: test-tool-qt4.cpp
	$(MOC) -o $@ $<

#../robot-test-tool: $(SIM_R) $(TWND_R)
#../robot-test-tool_d: $(SIM_D) $(TWND_D)

../robot-test-tool: $(SIM_R) $(TWND_R)
	$(LINK) $(LINK_OUT)$@ $^ $(LIBS)
../robot-test-tool_d: $(SIM_D) $(TWND_D)
	$(LINK) $(LINK_OUT)$@ $^ $(LIBS)

../cpp_household.so: $(SIM_R_PY) $(PYTH_R)
	$(LINK) $(SHARED) $(LINK_OUT)$@ $^ $(LIBS) $(LIBS_PY) $(BOOST_PYTHON)
../cpp_household_d.so: $(SIM_D) $(PYTH_D)
	$(LINK) $(SHARED) $(LINK_OUT)$@ $^ $(LIBS) $(LIBS_PY) $(BOOST_PYTHON)

../libroboschool.so: $(SIM_R) $(CPP_R)
	$(LINK) $(SHARED) $(LINK_OUT)$@ $^ $(LIBS)

../libroboschool_d.so: $(SIM_D) $(CPP_D)
	$(LINK) $(SHARED) $(LINK_OUT)$@ $^ $(LIBS)

$(OBJDIRR)/%.o: %.cpp
	$(CC) $(CFLAGS) -c $<  $(MINUS_O)$@ $(DEPENDS)
$(OBJDIRD)/%.o: %.cpp
	$(CC) $(CFLAGSD) -c $<  $(MINUS_O)$@ $(DEPENDS)

$(OBJDIRR_PY)/%.o: %.cpp
	$(CC) $(CFLAGS_PY) -c $<  $(MINUS_O)$@ $(DEPENDS)
$(OBJDIRD_PY)/%.o: %.cpp
	$(CC) $(CFLAGSD_PY) -c $<  $(MINUS_O)$@ $(DEPENDS)

.PHONY: depends clean dirs

clean:
	$(RM) $(EVERY_BIN) $(EVERY_OBJ_R) $(EVERY_OBJ_D) $(EVERY_OBJ_R_PY) $(EVERY_OBJ_D_PY) $(PYTH_R) $(PYTH_D) $(CPP_R) $(CPP_D) .generated/*.moc *.ilk *.pdb $(DEP) $(DEP_PY)
	rm -rf .generated
	rm -rf $(OBJDIRD)
	rm -rf $(OBJDIRR)
	rm -rf $(OBJDIRD_PY)
	rm -rf $(OBJDIRR_PY)

depends:
	cat  $(DEP) > Makefile.dep
	cat  $(DEP_PY) > Makefile.dep

.generated:
	mkdir -p .generated
$(OBJDIRR):
	mkdir -p $@
$(OBJDIRD):
	mkdir -p $@
$(OBJDIRR_PY):
	mkdir -p $@
$(OBJDIRD_PY):
	mkdir -p $@

dirs: .generated $(OBJDIRR) $(OBJDIRD) $(OBJDIRR_PY) $(OBJDIRD_PY)


-include Makefile.dep
