# Makefile automatically generated by ghdl
# Version: GHDL 0.34-dev (2016-09-14-339-g98892f0) [Dunoon edition] - mcode code generator
# Command used to generate this makefile:
# ghdl --gen-makefile --std=08 --workdir=work mac

GHDL=ghdl
GHDLFLAGS= --std=08 --workdir=work
GHDLRUNFLAGS=

# Default target : elaborate
all : elab

# Elaborate target.  Almost useless
elab : force
	$(GHDL) -c $(GHDLFLAGS) -e mac

# Run target
run : force
	$(GHDL) -c $(GHDLFLAGS) -r mac $(GHDLRUNFLAGS)

# Targets to analyze libraries
init: force
	# /usr/local/lib/ghdl/v08/std/../../src/std/textio.v08
	# /usr/local/lib/ghdl/v08/std/../../src/std/textio_body.v08
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/std_logic_1164.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/std_logic_1164-body.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/numeric_std.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/numeric_std-body.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/fixed_float_types.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/fixed_generic_pkg.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/math_real.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/math_real-body.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/fixed_generic_pkg-body.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/fixed_pkg.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/math_complex.vhdl
	# /usr/local/lib/ghdl/v08/ieee/../../src/ieee2008/math_complex-body.vhdl
	$(GHDL) -a $(GHDLFLAGS) mac.vhd

force:
