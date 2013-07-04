library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_serializer is
  port (
  clk:  out std_logic;
  data: out std_logic
  );
end entity tb_serializer;

architecture sim of tb_serializer is

  constant period: time := 234.12363 ns;
  signal w_clk: std_logic := '1';
  signal w_rst: std_logic := '1';

  component serializer is
  port (
    clk: in std_logic;
    rst: in std_logic;
    clkout: out std_logic;

    sdata:  out std_logic;
    -- Interface for FIFO
    data_in: in std_logic_vector(8 downto 0);
    data_read: out std_logic;
    data_avail: in std_logic
  );
  end component;

begin

  w_clk <= not w_clk after period/2;


  process
  begin
    wait for period/2;
    w_rst<='1';
    wait for period;
    w_rst<='0';
    wait;
  end process;


  ser: serializer
    port map (
      clk => w_clk,
      rst => w_rst,
      data_in => (others => 'X'),
      sdata => data,
      clkout => open,
      data_avail => '0'
    );

  
end sim;
