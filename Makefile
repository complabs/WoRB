###############################################################################
#
#  @file      Makefile
#  @brief     Makefile for Lab4: World of Rigid Bodies Test-Bed
#  @author    Mikica Kocic
#  @version   0.1
#  @date      2012-05-08
#  @copyright GNU Public License.
#

###############################################################################
# If you have nonstandard freeglut library, add -I<include> and -L<lib> to it !

GLUT_INC  := 
GLUT_LIB  := -lglut -lGLU -lGL

###############################################################################
# For verbose compilation (to see exectued commands) call make as: make Q=

Q := @

###############################################################################

CXXFLAGS  := -Wall $(GLUT_INC)
LDFLAGS   := $(GLUT_LIB)

SRC_DIR   := src
OBJ_DIR   := obj
BIN_DIR   := bin

###############################################################################

SRC_FILES := \
    Constants.cpp WoRB.cpp \
    CollisionDetection.cpp ImpulseMethod.cpp PositionProjections.cpp \
    Platform.cpp Utilities.cpp WoRB_TestBed.cpp Main.cpp

###############################################################################

SRC := $(addprefix $(SRC_DIR)/,$(SRC_FILES))
OBJ := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC))

vpath %.cpp $(SRC_DIR)

#CXXFLAGS  := $(CXXFLAGS) $(addprefix -I,$(SRC_DIR))

.PHONY: all rebuild clean distclean

###############################################################################

all : $(BIN_DIR)/WoRB

$(BIN_DIR)/WoRB : $(OBJ)
	@$(if $(Q), echo " [LD   ] " $@ )
	$(Q)g++ $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean :
	@$(if $(Q), echo " [RM   ] " $(OBJ_DIR)/\*.o )
	$(Q)rm -rf $(OBJ_DIR)/*.o

distclean : clean
	@$(if $(Q), echo " [RM   ] " $(BIN_DIR)/WoRB )
	$(Q)rm -rf $(BIN_DIR)/WoRB

rebuild : clean all

###############################################################################
# Dependencies (manual :)

Constants.o: Constants.cpp \
    Constants.h Quaternion.h

CollisionDetection.o: CollisionDetection.cpp \
    WoRB.h Constants.h Quaternion.h QTensor.h \
    Geometry.h RigidBody.h Collision.h CollisionResolver.h

ImpulseMethod.o: ImpulseMethod.cpp \
    WoRB.h Constants.h Quaternion.h QTensor.h \
    Geometry.h RigidBody.h Collision.h CollisionResolver.h

PositionProjections.o: PositionProjections.cpp \
    WoRB.h Constants.h Quaternion.h QTensor.h \
    Geometry.h RigidBody.h Collision.h CollisionResolver.h

WoRB.o: WoRB.cpp \
    WoRB.h Constants.h Quaternion.h QTensor.h \
    Geometry.h RigidBody.h Collision.h CollisionResolver.h

Platform.o: Platform.cpp

Utilities.o: Utilities.cpp \
    WoRB.h Constants.h Quaternion.h QTensor.h \
    Geometry.h RigidBody.h Collision.h CollisionResolver.h \
    Utilities.h WoRB_TestBed.h

WoRB_TestBed.o: WoRB_TestBed.cpp \
    WoRB.h Constants.h Quaternion.h QTensor.h \
    Geometry.h RigidBody.h Collision.h CollisionResolver.h \
    Utilities.h WoRB_TestBed.h

Main.o: Main.cpp \
    WoRB.h Constants.h Quaternion.h QTensor.h \
    Geometry.h RigidBody.h Collision.h CollisionResolver.h \
    Utilities.h WoRB_TestBed.h

###############################################################################
# Make goal definitions for each directory in OBJ_DIR

define make-goal
$1/%.o: %.cpp
	@$(if $(Q), echo " [C++  ] " $$@ )
	$(Q)g++ $(CXXFLAGS) -c $$< -o $$@
endef

$(foreach bdir,$(OBJ_DIR),$(eval $(call make-goal,$(bdir))))

###############################################################################
