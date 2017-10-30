COMPILER  = g++
CFLAGS    = -g -MMD -MP -Wall -Wextra -Winit-self -Wno-sign-compare -Wno-format -Wno-missing-field-initializers -std=c++0x 
ifeq "$(shell getconf LONG_BIT)" "64"
  LDFLAGS = -L/usr/lib64
else
  LDFLAGS =
endif
LIBS      = -lboost_system -lboost_thread -lboost_math_c99
INCLUDE   = -I./include -I../r_smth/src 
MY_OBJECT = $(wildcard ../r_smth/obj/*.o)
#TARGET    = ./a.out
SRCDIR    = ./src
ifeq "$(strip $(SRCDIR))" ""
  SRCDIR  = .
endif
SOURCES   = $(wildcard $(SRCDIR)/*.cpp)
OBJDIR    = ./obj
ifeq "$(strip $(OBJDIR))" ""
  OBJDIR  = .
endif
OBJECTS   = $(addprefix $(OBJDIR)/, $(notdir $(SOURCES:.cpp=.o)))
DEPENDS   = $(OBJECTS:.o=.d)

dummy: $(OBJECTS) $(LIBS)
	echo

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	-mkdir -p $(OBJDIR)
	$(COMPILER) $(CFLAGS) $(INCLUDE) -o $@ -c $<

all: clean $(TARGET)

clean:
	-rm -f $(OBJECTS) $(DEPENDS) $(TARGET)

-include $(DEPENDS)
