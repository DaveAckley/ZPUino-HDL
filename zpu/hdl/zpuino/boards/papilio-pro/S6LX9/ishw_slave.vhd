library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

library work;
use work.zpu_config.all;
use work.zpuino_config.all;
use work.zpuinopkg.all;
use work.zpupkg.all;
use work.wishbonepkg.all;

entity ishw_slave is
  port (
    wb_clk_i: in std_logic;
	 	wb_rst_i: in std_logic;
    wb_dat_o: out std_logic_vector(wordSize-1 downto 0);
    wb_dat_i: in std_logic_vector(wordSize-1 downto 0);
    wb_adr_i: in std_logic_vector(maxIObit downto minIObit);
    wb_we_i:  in std_logic;
    wb_cyc_i: in std_logic;
    wb_stb_i: in std_logic;
    wb_ack_o: out std_logic;
    wb_inta_o: out std_logic;
    id:       out slot_id;

    -- Wishbone MASTER interface
    mi_wb_dat_i: in std_logic_vector(wordSize-1 downto 0);
    mi_wb_dat_o: out std_logic_vector(wordSize-1 downto 0);
    mi_wb_adr_o: out std_logic_vector(maxAddrBitIncIO downto 0);
    mi_wb_sel_o: out std_logic_vector(3 downto 0);
    mi_wb_cti_o: out std_logic_vector(2 downto 0);
    mi_wb_we_o:  out std_logic;
    mi_wb_cyc_o: out std_logic;
    mi_wb_stb_o: out std_logic;
    mi_wb_ack_i: in std_logic;
    mi_wb_stall_i: in std_logic;

    -- signals to data lines
    scki_sw:      in std_logic;
    mosi_sw:      in std_logic;
    miso_sw:      out std_logic
  );
end entity ishw_slave;

architecture behave of ishw_slave is

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

  signal ddatavalid: std_logic;
  signal dclkout, dframedetected: std_logic;
  signal ddataout: std_logic_vector(7 downto 0);
  signal clkpulse: std_logic;

  -- Clock detection logic
  signal clkcount: unsigned(8 downto 0);
  signal clkperiod: unsigned(8 downto 0);
  signal clkdetected: std_logic;

  signal frame: std_logic;
  signal datavalid: std_logic;

  signal framedatacount: integer; -- NOTE: limit this

  type state_type is (
    idle,
    framecontrol,
    framedata
  );

  signal state: state_type;
  signal linkup: std_logic;

begin

  plck: pulse port map ( pulse_in => dclkout, pulse_out => clkpulse, clk => wb_clk_i, rst => wb_rst_i );
  plfr: pulse port map ( pulse_in => dframedetected, pulse_out => frame, clk => wb_clk_i, rst => wb_rst_i );
  pldt: pulse port map ( pulse_in => ddatavalid, pulse_out => datavalid, clk => wb_clk_i, rst => wb_rst_i );
  -- Alvie: can we assume that after pldt (datavalid pulse) data is indeed valid ? Think so...

  -- Clock counter
  process(wb_clk_i)
  begin
    if rising_edge(wb_clk_i) then
      if wb_rst_i='1' then
        clkcount<=(others => '0');
        clkperiod<=(others => '0');
        clkdetected<='0';
      else
        if clkpulse='1' then
          clkcount<=(others => '0');
          clkperiod <= clkcount;
          clkdetected <= '1';
        else
          if clkcount/=x"ff" then
            clkcount <= clkcount + 1;
          else
            clkdetected<='0';
          end if;
        end if;
      end if;
    end if;
  end process;


  ds: deserializer
    port map (
      clk => scki_sw,
      data => mosi_sw,
      datavalid => ddatavalid,
      dataout => ddataout,
      clkout => dclkout,
      framedetected => dframedetected
    );

  -- LLC (Link Layer control)

  process(wb_clk_i)
  begin
    if rising_edge(wb_clk_i) then
      if wb_rst_i='1' then
        framedatacount<=0;
        state <= idle;
        linkup <= '0';
      else
        case state is
          when idle =>
            if frame='1' then -- Signalled a frame.
              state <= framecontrol;
            end if;
          when framecontrol =>
            if frame='1' then -- Hmm, an empty frame it seems ?
              null;
            else
              if datavalid='1' then
                if linkup='1' then
                  state <= framedata;
                else
                  state <= framedata;
                end if;
              end if;
            end if;

            if clkdetected='0' then
              -- We lost clock.
              linkup <= '0';
              state <= idle;
            end if;

          when others =>

        end case;
        --
      end if;
    end if;
  end process;

end architecture;
