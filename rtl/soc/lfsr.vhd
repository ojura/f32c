library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

package fixed_point is
constant C_fracpart : integer := 20;
subtype fixed is signed(31 downto 0);
constant INV_PI : fixed := to_signed(16#517CC1B#, 32);
function mul_fixed(x, y : fixed) return fixed;
function ftrig(x : fixed; cos : boolean) return fixed;
end fixed_point;

package body fixed_point is

function mul_fixed(x, y : fixed) return fixed is
variable r : signed;
begin
	r := x * y;
	return r(31 + C_fracpart downto C_fracpart);
end mul_fixed;

-- 5th order sine approximation adapted from fixedpoint.h
function ftrig(x : fixed; cos : boolean) return fixed is
variable x1, x2, y : fixed;
constant shift1 : integer := 4;

constant B : fixed := to_signed(16#243f6a#, 32);  -- (M_PI - 3) << shift1
constant C : fixed := to_signed(16#1487ed5#, 32); -- (2*M_PI - 5) << shift1
constant D : fixed := to_signed(16#3243f6a#, 32); -- M_PI << shift1

constant qN : integer := C_fracpart;
variable r : signed;

begin
	r := x * INV_PI;
    x1 := r(59 downto 28);
    
    x1 := shift_left(x1,(30-qN-shift1)); -- shift to full s32 range
    
    if(cos) then 
		x1 := x1 + shift_left(to_signed(1, 32), 30);
	end if;
    
    if (x1(31) xor x1(30)) = '1' then     -- test for quadrant 1 or 2
        x1 := shift_left(to_signed(1, 32), 31) - x1;
    end if;
	
    x1 := shift_right(x1, (30-qN-shift1));
    
    x2 := mul_fixed(x1, x1);
    x2 := shift_right(x2, shift1);
    y := C - shift_right(mul_fixed(x2,B), shift1);
    y := D - shift_right(mul_fixed(x2,y), shift1);
    y := mul_fixed(x1, y);
    
    return shift_right(y, (2*shift1+1));

end ftrig;
end fixed_point;

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;
use work.fixed_point.all;

entity fsin is
    port (
	ce, clk: in std_logic;
	bus_in: in std_logic_vector(31 downto 0);
	bus_write: in std_logic;
	addr: in std_logic;
	bus_out: out std_logic_vector(31 downto 0);
	fsin_ready: out std_logic
    );
end fsin;

architecture x of fsin is
	signal res: fixed;
	signal R_theta, theta, theta_cos: std_logic_vector(12 downto 0);
	signal clken_sincos: std_logic;
	
	signal sin : signed( 15 downto 0);
	
	constant wait_cycles : integer := 0; 
	
	signal R_wait : integer range 0 to wait_cycles;
begin

	theta <= std_logic_vector( shift_right((signed(bus_in) * INV_PI), 49-13)(12 downto 0) );
	theta_cos <= (theta(12 downto 11) + 1) & theta(10 downto 0);

    process(clk)
    begin
    if rising_edge(clk) then	
		if wait_cycles > 0 and R_wait < wait_cycles then
			R_wait <= R_wait + 1;
		end if;
		if ce = '1' and bus_write = '1' then
	
			if addr = '0' then 
				R_theta <= theta;
			else
				R_theta <= theta_cos;
			end if;
			
			if wait_cycles > 0 then R_wait <= 0; end if;
		end if;
    end if;
	end process;
	
	fsin_ready <= '1' when wait_cycles = 0 or R_wait = wait_cycles else '0';

    --bus_out <= std_logic_vector(ftrig(fixed(R_x), false));

	--bus_out <= "0000000000000000000000" & theta;
	
	bus_out <= std_logic_vector(sin) & x"0000";
	
	clken_sincos <= ce and bus_write;
	
-- parameterized module component instance
   I_sincos: entity work.sintab
	 generic map ( pipestages => 0 )
     port map (clk=>clk, theta=> unsigned(R_theta), sine=>sin);
	
end x;


library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

entity lfsr is
    port (
	ce, clk: in std_logic;
	addr: in std_logic;
	bus_in: in std_logic_vector(31 downto 0);
	bus_write: in std_logic;
	bus_out: out std_logic_vector(31 downto 0);
	lfsr_ready: out std_logic
    );
end lfsr;

architecture x of lfsr is
    signal R_state: std_logic_vector(30 downto 0) := std_logic_vector(to_unsigned(16#0#, 31)); -- := SEED;
begin

    process(clk)
	variable new_state : std_logic_vector(30 downto 0);
    begin
    if rising_edge(clk) then

		if ce = '1' then
			new_state := R_state;
			for i in 0 to 30 loop
				new_state := not((new_state(0) xor new_state(3)))& new_state(30 downto 1);
			end loop;
			R_state <= new_state;
		end if;
		
		if ce = '1' and bus_write = '1' then
			R_state <= bus_in(30 downto 0);
		end if;
		
    end if;
    end process;

	lfsr_ready <= '1';
    bus_out <= '0' & R_state;
	
end x;

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;
use work.fixed_point.all;

entity gauss is
    port (
	ce, clk: in std_logic;
	addr: in std_logic_vector(3 downto 2);
	bus_in: in std_logic_vector(31 downto 0);
	bus_write: in std_logic;
	bus_out: out std_logic_vector(31 downto 0);
	gauss_ready: out std_logic
    );
end gauss;

architecture x of gauss is
constant C_num_lfsrs : integer := 4;

type lfsr_ce_array is array(C_num_lfsrs-1 downto 0) of std_logic;
type from_lfsr_array is array(C_num_lfsrs-1 downto 0) of std_logic_vector(31 downto 0);


signal lfsr_ce, lfsr_decoded: lfsr_ce_array;
signal from_lfsr : from_lfsr_array;
signal addr_int : integer range 0 to 3;

signal R_lfsr_write_counter: std_logic_vector(1 downto 0);

signal R_arg_mean : fixed;
signal R_arg_stdev : fixed;

begin

	gauss_ready <= '1';
	
	-- enable shifting all LFSRs when reading, enable only one when initializing
	lfsr_ce <= lfsr_decoded when ce = '1' and addr = "01" and bus_write = '1' else "1111" when ce = '1' and bus_write ='0' else "0000";

	process(clk) 
	begin
	if rising_edge(clk) then 
		if ce = '1' and bus_write = '1' then
		-- write: 	addr = 00: reset init counter
		--			addr = 01: write to current lfsr
		--			addr = 10: write mean argument
		--			addr = 11: write stddev argument
			case addr is
				when "00" => R_lfsr_write_counter <= "00";
				when "01" => R_lfsr_write_counter <= R_lfsr_write_counter + 1;
				when "10" => R_arg_mean <= fixed(bus_in);
				when "11" => R_arg_stdev <= fixed(bus_in);
			end case;
		end if;		
	end if;
	end process;
	
	addr_int <= to_integer(unsigned(R_lfsr_write_counter));
	
	process(addr_int)
	variable lfsr_decoded_var : lfsr_ce_array;
	begin
		lfsr_decoded_var := (others => '0');
		lfsr_decoded_var( addr_int ) := '1';
		lfsr_decoded <= lfsr_decoded_var;
	end process;
	
	process(from_lfsr)
	variable uniforms : from_lfsr_array;
	variable sum : fixed;
	-- norm: normalization constant which ensures stdev = 1  
	constant norm : fixed := to_signed(7094, 32);
	
	variable r : signed(63 downto 0);
	begin
		for i in 0 to C_num_lfsrs -1 loop
			uniforms(i) := from_lfsr(i)(29) & from_lfsr(i)(29) & from_lfsr(i)(29 downto 0);
		end loop;	
		sum := signed(uniforms(0) + uniforms(1) + uniforms(2) + uniforms(3));
		--r = r * ((fixed(sigma << 2) * fixed( 1/(6.1993e+08 / (1<<FIXED_FRACPART)) * (1<<2))) >> 4) + mi; 
		
		r :=  (R_arg_stdev * norm);
		r := sum * r(31 + C_fracpart + 2 downto C_fracpart + 2);
		
		bus_out <= std_logic_vector( r(31 + C_fracpart downto C_fracpart) + R_arg_mean );

		
	end process;


   I_lfsrs: for i in 0 to C_num_lfsrs-1 generate
   begin
      lfsr : entity work.lfsr
	  port map (
	  clk => clk, ce => lfsr_ce(i), addr => open,
	bus_out => from_lfsr(i), bus_write => bus_write, bus_in => bus_in, lfsr_ready => open
    );
   end generate;

end x;


----viseclockni, jednostavni LFSR trosi vise  LUTova! (?!)
--library ieee;
--use ieee.std_logic_1164.all;
--use ieee.std_logic_unsigned.all;
--use ieee.numeric_std.all;

--entity lfsr is
	----generic ( SEED : STD_LOGIC_VECTOR(30 downto 0):= (others => '0');
    --port (
	--ce, clk: in std_logic;
	--addr: in std_logic;
	--bus_in: in std_logic_vector(31 downto 0);
	--bus_write: in std_logic;
	--bus_out: out std_logic_vector(31 downto 0);
	--lfsr_ready: out std_logic
    --);
--end lfsr;

--architecture x of lfsr is
    --signal R_state: std_logic_vector(30 downto 0) := std_logic_vector(to_unsigned(16#0#, 31)); -- := SEED;
	--signal R_counter: std_logic_vector(4 downto 0) := std_logic_vector(to_unsigned(0, 5));
	--signal counter_ready: std_logic;
--begin

	--counter_ready <= '1' when R_counter = std_logic_vector(to_unsigned(0, 5)) else '0';

    --process(clk)
    --begin
    --if rising_edge(clk) then
		--if counter_ready = '0' then
			--R_state <= not((R_state(0) xor R_state(3)))& R_state(30 downto 1);
			--R_counter <= R_counter - 1;
		--end if;
		
		--if ce = '1' and addr = '0' and counter_ready = '1' then
			--R_counter <= std_logic_vector(to_unsigned(31, 5));
		--end if;
		
		--if ce = '1' and bus_write = '1' then
			--R_counter <= std_logic_vector(to_unsigned(0, 5));
			--R_state <= bus_in(30 downto 0);
		--end if;
		
    --end if;
    --end process;

	--lfsr_ready <= counter_ready or addr;
    --bus_out <= '0' & R_state when addr = '0' else std_logic_vector(resize(unsigned(R_counter), 32));
	
--end x;
