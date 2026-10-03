#
# TODO: none
#

# Specify the main compiler:
CC := g++
# CC := clang --analyze

# Specify the source and build directories
SRCDIR := src
BUILDDIR := build

# Auto-generate a list of all header files
# > Specify the extension of header files
HEADEREXT := h
# > Find all files with this extension in the source directory
HEADERS := $(shell find $(SRCDIR) -type f -name *.$(HEADEREXT))
# > Echo the list of sources
$(info List of header files found:)
$(info > $(HEADERS))
$(info )

# Auto-generate a list of all C++ source files
# > Specify the extension of source files
SRCEXTCPP := cpp
# > Find all files with this extension in the source directory
SOURCESCPP := $(shell find $(SRCDIR) -type f -name *.$(SRCEXTCPP))
# > Echo the list of sources
$(info List of C++ source files found:)
$(info > $(SOURCESCPP))
$(info )


# Auto-generate a list of all C source files
# > Specify the extension of source files
SRCEXTC := c
# > Find all files with this extension in the source directory
SOURCESC := $(shell find $(SRCDIR) -type f -name *.$(SRCEXTC))
# > Echo the list of sources
$(info List of C source files found:)
$(info > $(SOURCESC))
$(info )

# Auto-generate a list of all object files
# to be compiled from C++ code.
# > The same as the "SOURCESCPP" list but with
#   the extension and directory changed.
# > ".cpp" is replaced with ".o" by using the
#   modifier ".$(SRCEXTCPP)=.o"
# > "$(SRCDIR)/" is repalced with "$(BUILDDIR)/"
#   by using the "patsubstr" function ("pattern
#   substitute string"), see:
#   https://www.gnu.org/software/make/manual/html_node/Text-Functions.html
#   for documentation of the function
OBJECTSCPP := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCESCPP:.$(SRCEXTCPP)=.o))
#TEMPSRCNODIR := $(notdir $(SOURCESCPP))
#OBJECTSCPP := $(addprefix $(BUILDDIR)/,$(TEMPSRCNODIR:.$(SRCEXTCPP)=.o))
# > Echo the list of objects
$(info List of objects to be generated from C++ code:)
$(info > $(OBJECTSCPP))
$(info )
# Ensure that a directory exists for all the objects
$(shell mkdir -p $(dir $(OBJECTSCPP)))

# Auto-generate a list of all object files
# to be compiled from C-code.
# > The same as the "SOURCESC" list but with
#   the extension and directory changed.
OBJECTSC := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCESC:.$(SRCEXTC)=.o))
# > Echo the list of objects
$(info List of objects to be generated from C code:)
$(info > $(OBJECTSC))
$(info )
# Ensure that a directory exists for all the objects
$(shell mkdir -p $(dir $(OBJECTSC)))

# Specify the test source code and target directories
TESTSRCDIR := test
TARGETDIR := bin

# Ensure that a directory exists for the targets
$(shell mkdir -p $(TARGETDIR))


# Auto-generate a list of all test source files
# > Specify the extension of source files
TESTSRCEXT := cpp
# > Find all files with this extension in the source directory
TESTSOURCES := $(shell find $(TESTSRCDIR) -type f -name *.$(TESTSRCEXT))
# > Echo the list of sources
$(info List of test source files found:)
$(info > $(TESTSOURCES))
$(info )

# Auto-generate a list of all test object
# files to be compiled. The same as the
# "TESTSOURCES" list but with the extension
# and directory changed.
TESTOBJECTS := $(patsubst $(TESTSRCDIR)/%,$(BUILDDIR)/%,$(TESTSOURCES:.$(TESTSRCEXT)=.o))
# > Echo the list of objects
$(info List of test objects to be generated:)
$(info > $(TESTOBJECTS))
$(info )

# Auto-generate a list of all targets.
# The same as the "TESTSOURCES" list
# but with the extension removed and
# the directory changed.
TARGETS := $(patsubst $(TESTSRCDIR)/%,$(TARGETDIR)/%,$(TESTSOURCES:.$(TESTSRCEXT)=))
# > Echo the list of objects
$(info List of targets to be generated:)
$(info > $(TARGETS))
$(info )

# Specify the target binary file
#TARGET := bin/test


#$(TARGET): $(BUILDDIR)/test.o $(BUILDDIR)/i2c_driver.o $(BUILDDIR)/pololu_smc_g2.o
#	$(CC) -o $(TARGET) $(BUILDDIR)/test.o $(BUILDDIR)/i2c_driver.o
#	@echo " Linking..."

all: $(TARGETS)

$(TARGETS): $(TESTOBJECTS) $(OBJECTSCPP) $(OBJECTSC)
	$(CC) -o $@ $(subst $(TARGETDIR),$(BUILDDIR),$@).o $(OBJECTSCPP) $(OBJECTSC) -lgpiod -lpthread
	@echo " Linking..."

#$(TARGET): $(BUILDDIR)/test.o $(BUILDDIR)/i2c_driver.o $(BUILDDIR)/pololu_smc_g2.o
#	$(CC) -o $(TARGET) $(BUILDDIR)/test.o $(BUILDDIR)/i2c_driver.o
#	@echo " Linking..."



$(TESTOBJECTS) : $(patsubst $(BUILDDIR)/%,$(TESTSRCDIR)/%,$(@:.o=.$(SRCEXTCPP))) $(HEADERS)
	$(CC) -o $@ -c $(patsubst $(BUILDDIR)/%,$(TESTSRCDIR)/%,$(@:.o=.$(SRCEXTCPP))) -I$(SRCDIR)

#$(BUILDDIR)/test.o : $(SRCDIR)/test.cpp $(SRCDIR)/i2c_driver/i2c_driver.h
#	$(CC) -o $(BUILDDIR)/test.o -c $(SRCDIR)/test.cpp -I$(SRCDIR)/i2c_driver

$(OBJECTSCPP) : $(patsubst $(BUILDDIR)/%,$(SRCDIR)/%,$(@:.o=.$(SRCEXTCPP))) $(patsubst $(BUILDDIR)/%,$(SRCDIR)/%,$(@:.0=.h))
	$(CC) -o $@ -c $(patsubst $(BUILDDIR)/%,$(SRCDIR)/%,$(@:.o=.$(SRCEXTCPP))) -I$(SRCDIR)

$(OBJECTSC) : $(patsubst $(BUILDDIR)/%,$(SRCDIR)/%,$(@:.o=.$(SRCEXTC))) $(patsubst $(BUILDDIR)/%,$(SRCDIR)/%,$(@:.0=.h))
	$(CC) -o $@ -c $(patsubst $(BUILDDIR)/%,$(SRCDIR)/%,$(@:.o=.$(SRCEXTC))) -I$(SRCDIR)

#$(BUILDDIR)/i2c_driver.o : $(SRCDIR)/i2c_driver/i2c_driver.cpp $(SRCDIR)/i2c_driver/i2c_driver.h
#	$(CC) -o $(BUILDDIR)/i2c_driver.o -c $(SRCDIR)/i2c_driver/i2c_driver.cpp -I$(SRCDIR)/i2c_driver

#$(BUILDDIR)/pololu_smc_g2.o : $(SRCDIR)/pololu_smc_g2/pololu_smc_g2.cpp $(SRCDIR)/pololu_smc_g2/pololu_smc_g2.h $(SRCDIR)/i2c_driver/i2c_driver.h
#	$(CC) -o $(BUILDDIR)/pololu_smc_g2.o -c $(SRCDIR)/pololu_smc_g2/pololu_smc_g2.cpp -I$(SRCDIR)/pololu_smc_g2 -I$(SRCDIR)/i2c_driver

clean:
	@echo " Cleaning the build directory and targets:"
	@echo " rm -rf $(BUILDDIR)/* $(TARGETS)"; rm -rf $(BUILDDIR)/* $(TARGETS)

.PHONY: clean


