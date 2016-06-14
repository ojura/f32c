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

entity gauss is
    port (
	ce, clk: in std_logic;
	addr: in std_logic_vector(3 downto 2);
	bus_in: in std_logic_vector(31 downto 0);
	bus_write: in std_logic;
	bus_out: out std_logic_vector(31 downto 0);
	gauss_ready: out std_logic;
	led: out std_logic_vector(7 downto 0)
    );
end gauss;

architecture x of gauss is
constant C_num_lfsrs : integer := 4;
constant C_fracpart : integer := 20;

subtype fixed is signed(31 downto 0);

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
				when "10" => R_arg_mean <= bus_in;
				when "11" => R_arg_stdev <= bus_in;
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
	constant norm : fixed := std_logic_vector(to_signed(7094, 32));
	
	variable r : fixed;
	begin
		for i in 0 to C_num_lfsrs -1 loop
			uniforms(i) := from_lfsr(i)(29) & from_lfsr(i)(29) & from_lfsr(i)(29 downto 0);
		end loop;	
		sum := signed(uniforms(0) + uniforms(1) + uniforms(2) + uniforms(3));
		--r = r * ((fixed(sigma << 2) * fixed( 1/(6.1993e+08 / (1<<FIXED_FRACPART)) * (1<<2))) >> 4) + mi; 
		
		r :=  (R_arg_stdev * norm)(31+ C_fracpart + 2 downto C_fracpart + 2);
		r := (sum * r)(31+ C_fracpart downto C_fracpart) + R_arg_mean;
		bus_out <= r;
		
	end process;


   I_lfsrs: for i in 0 to C_num_lfsrs-1 generate
   begin
      lfsr : entity work.lfsr
	  port map (
	  clk => clk, ce => lfsr_ce(i), addr => open,
	bus_out => from_lfsr(i), bus_write => bus_write, bus_in => bus_in, lfsr_ready => open
    );
   end generate;
   
   led <= "000000" & std_logic_vector(R_lfsr_write_counter);

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
