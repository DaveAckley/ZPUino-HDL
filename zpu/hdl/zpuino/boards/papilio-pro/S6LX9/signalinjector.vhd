library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity signalinjector is
  port (
  clk:  out std_logic;
  data: out std_logic
  );
end entity signalinjector;

architecture sim of signalinjector is

  constant period: time := 234.12363 ns;
  signal w_clk: std_logic := '1';

  signal shiftreg: std_logic_vector(65 downto 0) :=
    "01111111" &
    "01010101" &
    "01100110" &

    "01010101" &
    "011111101" & -- escaped frame sequence '01111111';
    "01100110" &
    "011111100" & -- True '01111110'
    "01111111";

begin

  w_clk <= not w_clk after period/2;
  clk <= w_clk;

  process(w_clk)
  begin
    if rising_edge(w_clk) then
      shiftreg(shiftreg'HIGH downto 1) <= shiftreg(shiftreg'HIGH-1 downto 0);
      shiftreg(0) <= shiftreg(shiftreg'HIGH);
    end if;
    if falling_edge(w_clk) then
      data <= shiftreg(shiftreg'HIGH);
    end if;
  end process;

end sim;
