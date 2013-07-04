library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_frame_tx is
end entity tb_frame_tx;

architecture sim of tb_frame_tx is

  constant period: time := 20.12363 ns;
  constant sysperiod: time := 10.33 ns;

  signal w_clk: std_logic := '1';
  signal w_txclk: std_logic := '1';
  signal w_rst: std_logic := '1';
  signal w_txrst: std_logic := '1';
  signal clk, data:std_logic;
  component frame_tx is
  port (
    -- Host clocks
    clk:  in std_logic;
    rst:  in std_logic;
    -- FIFO interface for host
    data_in: in std_logic_vector(32 downto 0);
    data_wr: in std_logic;
    full:     out std_logic;
    empty:    out std_logic;

    -- Serial clocks and data
    sclk: in std_logic;
    arst: in std_logic;
    clkout: out std_logic;

    sdata:  out std_logic
  );
  end component;

  component deserializer is
  port (
    clk: in std_logic;
    data:in std_logic;
    datavalid: out std_logic;
    dataout: out std_logic_vector(7 downto 0);
    clkout: out std_logic;
    framedetected: out std_logic
  );
  end component;

  signal data_write: std_logic_vector(32 downto 0);
  signal data_write_enable: std_logic := '0';

begin



  w_clk <= not w_clk after sysperiod/2;
  w_txclk <= not w_txclk after period/2;

  process(w_rst, w_txclk)
  begin
    if w_rst='1' then
      w_txrst<='1';
    else
      if rising_edge(w_txclk) then
        w_txrst<='0';
      end if;
    end if;
  end process;

  process
  begin
    wait for sysperiod/2;
    w_rst<='1';
    wait for sysperiod;
    w_rst<='0';
    wait;
  end process;

  ftx: frame_tx
  port map (
    -- Host clocks
    clk       => w_clk,
    rst       => w_rst,
    -- FIFO interface for host
    data_in   => data_write,
    data_wr   => data_write_enable,
    full      => open,
    empty     => open,

    -- Serial clocks and data
    sclk      => w_txclk,
    arst      => w_txrst,
    clkout    => clk,
    sdata     => data
  );

  ii: block
    signal cnt: integer :=0;
  begin
    process(w_clk)
    begin
      if rising_edge(w_clk) then
        cnt <= cnt + 1;
        data_write_enable<='0';
        case cnt is
          when 100 =>
            data_write<='1' & x"7EAAAA00";
            data_write_enable<='1';
          when 1600 =>
            data_write<='1' & x"FFFFFFFF";
            data_write_enable<='1';
          when others =>
        end case;
      end if;
    end process;
  end block;

  des: deserializer
  port map (
    clk     => clk,
    data    => data
  );

  
end sim;
