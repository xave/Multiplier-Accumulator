-- ghdl -a --std=08 --workdir=work mac.vhdl
library ieee;
use ieee.std_logic_1164.all, ieee.fixed_pkg.all;
use ieee.math_complex.all;

entity mac is 
    port( clk, reset : in std_ulogic;
          x_real : in u_sfixed(0 downto -15);  -- real and imaginary part of the two input data sequences
          x_imag : in u_sfixed(0 downto -15);
          y_real : in u_sfixed(0 downto -15);
          y_imag : in u_sfixed(0 downto -15);
          s_real : out u_sfixed(0 downto -15); -- real and imaginary parts of accumulated sum
          s_imag : out u_sfixed(0 downto -15);
          ovf : out std_ulogic); --overflow flag
end entity mac;

-- Behavioral model of MAC algorithm allows for focus on the algorithm without being distracted 
-- by other details at this erly design stage.

architecture behavioral of mac is 
    signal x_complex, y_complex, s_complex : complex;

begin
    x_complex <= ( to_real(x_real), to_real(x_imag) );
    y_complex <= ( to_real(y_real), to_real(y_imag) );

    behavior : process (clk) is
        variable input_x, input_y : complex := (0.0, 0.0);
        variable real_part_product_1, real_part_product_2,
                 imag_part_product_1, imag_part_product_2 : real := 0.0;   
        variable product, sum : complex := (0.0, 0.0);
        variable real_accumulator_ovf,
                 imag_accumulator_ovf : boolean := false;
    begin
        if rising_edge(clk) then
            -- Work from the end of the pipeline back to the start,
            -- so as not to overwrite previosu results from the pipeline
            -- registers before they are even used. 

            -- Update accumulator and generate outputs. 
            if reset then
                sum := (0.0, 0.0);
                real_accumulator_ovf := false;
                imag_accumulator_ovf := false;
            else
                sum := product + sum;
                real_accumulator_ovf := real_accumulator_ovf 
                                        or sum.re < -16.0
                                        or sum.re >= +16.0;
                imag_accumulator_ovf := imag_accumulator_ovf 
                                        or sum.im < -16.0
                                        or sum.im >= +16.0;
            end if;
            s_complex <= sum;
            ovf <= '1';
            ovf <= '1' when (real_accumulator_ovf or imag_accumulator_ovf
                            or sum.re < -1.0 or sum.re >= +1.0
                            or sum.im < -1.0 or sum.im >= +1.0 ) else '0';
            -- Update product registers
            product.re := real_part_product_1 - real_part_product_2;
            product.im := imag_part_product_1 + imag_part_product_2;

            -- Update partial product registers
            -- (actually with the full product).
            real_part_product_1 := input_x.re * input_y.re;
            real_part_product_2 := input_x.im * input_y.im;
            imag_part_product_1 := input_x.re * input_y.re;
            imag_part_product_2 := input_x.im * input_y.im;

            -- Update input registers using MAC inputs
            input_x := x_complex;
            input_y := y_complex;
        end if;
    end process behavior;
    s_real <= to_sfixed(s_complex.re, s_real);
    s_imag <= to_sfixed(s_complex.im, s_imag);
end architecture behavioral;

architecture rtl of mac is
   signal pipelined_x_real,
           pipelined_x_imag,
           pipelined_y_real,
           pipelined_y_imag : u_sfixed(0 downto -15);
   signal real_part_product_1,
           real_part_product_2,
           imag_part_product_1,
           imag_part_product_2 : u_sfixed(1 downto -30);
   signal pipelined_real_part_product_1,
           pipelined_real_part_product_2,
           pipelined_imag_part_product_1,
           pipelined_imag_part_product_2 : u_sfixed(1 downto -30);
   signal real_product,
           imag_product : u_sfixed(2 downto -30);
   signal pipelined_real_product,
           pipelined_imag_product : u_sfixed(2 downto -17);
   signal extended_real_product,
           extended_imag_product : u_sfixed(4 downto -17);
   signal real_sum,
           imag_sum : u_sfixed(4 downto -17);
   signal real_accumulator_ovf,
           imag_accumulator_ovf : std_ulogic;
    signal pipelined_real_sum,
            pipelined_imag_sum : u_sfixed(4 downto -17);
    signal pipelined_real_accumulator_ovf,
            pipelined_imag_accumulator_ovf : std_ulogic;
begin
   input_reg : process (clk) is
   begin 
       if rising_edge(clk) then
           pipelined_x_real <= x_real;
           pipelined_x_imag <= x_imag;
           pipelined_x_real <= y_real;
           pipelined_y_imag <= y_imag;
       end if;
   end process input_reg;

   real_part_product_1 <= pipelined_x_real * pipelined_y_real;
   real_part_product_2 <= pipelined_x_imag * pipelined_y_imag;

   imag_part_product_1 <= pipelined_x_real * pipelined_y_imag;
   imag_part_product_2 <= pipelined_x_imag * pipelined_y_real;
   
   part_product_reg : process (clk) is
   begin
       if rising_edge(clk) then
           pipelined_real_part_product_1 <= real_part_product_1;
           pipelined_real_part_product_2 <= real_part_product_2;
           pipelined_imag_part_product_1 <= imag_part_product_1;
           pipelined_imag_part_product_2 <= imag_part_product_2;
       end if;
   end process part_product_reg;

   real_product <= pipelined_real_part_product_1
                   - pipelined_real_part_product_2;
   imag_product <= pipelined_imag_part_product_1
                   - pipelined_imag_part_product_2;

   product_reg : process (clk) is 
   begin
       if rising_edge(clk) then
           pipelined_real_product
               <= resize(real_product, pipelined_real_product);
           pipelined_imag_product
               <= resize(imag_product, pipelined_imag_product);
       end if;
   end process product_reg;

   extended_real_product
       <= resize(pipelined_real_product, extended_real_product);
   extended_imag_product
       <= resize(pipelined_imag_product, extended_imag_product);
   real_sum <= extended_real_product + pipelined_real_sum;
   imag_sum <= extended_imag_product + pipelined_imag_sum;

   real_accumulator_ovf
       <= (    not extended_real_product(4)  -- non-negative
           and not pipelined_real_sum(4)     -- non-negative
           and     real_sum(4) )             -- appears negative
       or
          (    extended_real_product(4)      -- negative
           and     pipelined_real_sum(4)     -- negative
           and not real_sum(4) );            -- appears non-negative


   imag_accumulator_ovf
       <= (    not extended_imag_product(4)  -- non-negative
           and not pipelined_imag_sum(4)     -- non-negative
           and     imag_sum(4) )             -- appears negative
       or
          (    extended_imag_product(4)      -- negative
           and     pipelined_imag_sum(4)     -- negative
           and not imag_sum(4) );            -- appears non-negative

   accumulator_reg : process (clk) is
   begin
       if rising_edge(clk) then
           if reset then
               pipelined_real_sum <= (others => '0');
               pipelined_imag_sum <= (others => '0');
               pipelined_real_accumulator_ovf <= '0';
               pipelined_imag_accumulator_ovf <= '0';
           else
               pipelined_real_sum <= real_sum;
               pipelined_imag_sum <= imag_sum;
               pipelined_real_accumulator_ovf
                   <= pipelined_real_accumulator_ovf or real_accumulator_ovf;
               pipelined_imag_accumulator_ovf
                   <= pipelined_imag_accumulator_ovf or imag_accumulator_ovf;
           end if;
       end if;
   end process accumulator_reg;

   s_real <= resize(pipelined_real_sum, s_real);
   s_imag <= resize(pipelined_imag_sum, s_imag);
   ovf <= real_accumulator_ovf or imag_accumulator_ovf
           or pipelined_real_sum(4 downto 0) ?= "00000"
           or pipelined_real_sum(4 downto 0) ?= "11111"
           or pipelined_imag_sum(4 downto 0) ?= "00000"
           or pipelined_imag_sum(4 downto 0) ?= "11111";
   end architecture rtl;

