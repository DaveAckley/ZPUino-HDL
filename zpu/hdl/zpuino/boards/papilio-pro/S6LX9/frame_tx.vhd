library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
-- synthesis translate_off
use work.txt_util.all;
-- synthesis translate_on

entity frame_tx is
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
end entity frame_tx;

architecture sim of frame_tx is

  component gh_fifo_async_rrd_sr_wf is
	GENERIC (add_width: INTEGER :=8; -- min value is 2 (4 memory locations)
	         data_width: INTEGER :=8 ); -- size of data bus
	port (					
		clk_WR  : in STD_LOGIC; -- write clock
		clk_RD  : in STD_LOGIC; -- read clock
		rst     : in STD_LOGIC; -- resets counters
		srst    : in STD_LOGIC:='0'; -- resets counters (sync with clk_WR)
		WR      : in STD_LOGIC; -- write control 
		RD      : in STD_LOGIC; -- read control
		D       : in STD_LOGIC_VECTOR (data_width-1 downto 0);
		Q       : out STD_LOGIC_VECTOR (data_width-1 downto 0);
		empty   : out STD_LOGIC;
		qfull   : out STD_LOGIC;
		hfull   : out STD_LOGIC;
		qqqfull : out STD_LOGIC;
    afull   : out STD_LOGIC;
		full    : out STD_LOGIC);
  end component;

  component serializer is
  port (
    clk: in std_logic;
    rst: in std_logic;
    clkout: out std_logic;

    sdata:  out std_logic;
    -- Interface for FIFO
    data_in: std_logic_vector(32 downto 0);
    data_read: out std_logic;
    data_avail: in std_logic
  );
  end component;

  signal fifo_data_read: std_logic_vector(32 downto 0);
  signal fifo_data_write: std_logic_vector(32 downto 0);
  signal fifo_read_enable: std_logic;
  signal fifo_empty: std_logic;
  signal fifo_not_empty: std_logic;

  signal fifo_clear: std_logic;
  signal fifo_write_enable: std_logic;
  signal fifo_quad_full, fifo_half_full, fifo_almost_full, fifo_full: std_logic;

begin
  
  fifo_not_empty<=not fifo_empty;
  fifo_clear <= arst;
  full <= fifo_full;
  empty <= fifo_empty;
  fifo_data_write <= data_in;
  fifo_write_enable <= data_wr;

  ser: serializer
  port map (
    clk     => sclk,
    rst     => arst,
    clkout  => clkout,

    sdata   => sdata,
    -- Interface for FIFO
    data_in   => fifo_data_read,
    data_read => fifo_read_enable,
    data_avail=> fifo_not_empty
  );


  myfifo: gh_fifo_async_rrd_sr_wf
  generic map (
    data_width => 33,
    add_width => 4
  )
  port map (
		clk_WR  => clk,
		clk_RD  => sclk,
		rst     => fifo_clear,
		srst    => '0',--fifo_clear,
		WR      => fifo_write_enable,
		RD      => fifo_read_enable,
		D       => fifo_data_write,
		Q       => fifo_data_read,
		empty   => fifo_empty,
		qfull   => fifo_quad_full,
		hfull   => fifo_half_full,
		qqqfull => fifo_almost_full,
		full    => fifo_full
  );

end sim;

