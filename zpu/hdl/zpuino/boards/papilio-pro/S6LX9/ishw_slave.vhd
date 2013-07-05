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
    framedetected: out std_logic;
    idledetected: out std_logic
  );
  end component;

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
    framedata,
    queuewrite
  );

  signal state: state_type;
  signal linkup: std_logic;

  signal txfifo_in: std_logic_vector(32 downto 0);
  signal txfifo_we: std_logic;
  signal txfifo_empty, txfifo_full: std_logic;

  -- Slave configuration

  signal ien: std_logic; -- Interrupts enabled
  signal int: std_logic; -- Interrupt
  signal rxf: std_logic; -- RX finished
  signal rxe: std_logic; -- RX enabled
  signal txf: std_logic; -- TX finished

  subtype rxptr is std_logic_vector(maxAddrBitIncIO downto 8); -- 256-bytes aligned
  type rxarray_type is array(0 to 7) of rxptr;

  signal rxaddr: rxarray_type;
  signal rxindex: integer range 0 to 7;
  signal irxindex: integer range 0 to 7;

  signal txaddr: std_logic_vector(maxAddrBitIncIO downto 0); -- Source address for tx

  signal rxcount, rxsize, txcount: unsigned(31 downto 0);
  signal rxdata: std_logic_vector(31 downto 0);
  signal datacount: integer range 0 to 3;
  signal rxdata_q: std_logic_vector(31 downto 0);
  signal rxdatavalid: std_logic;
  signal writeinprogress: std_logic;

  signal currentoffset: std_logic_vector(7 downto 0);
  signal ack_i: std_logic;

begin

  id <= x"05" & x"01";
  wb_inta_o<=int;

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

  txs: frame_tx
    port map (
      clk   => wb_clk_i,
      rst   => wb_rst_i,
      -- FIFO interface for host
      data_in => txfifo_in,
      data_wr => txfifo_we,
      full    => txfifo_full,
      empty   => txfifo_empty,
  
      -- Serial clocks and data
      sclk    => scki_sw,
      arst    => wb_rst_i,
      clkout  => open,
      sdata   => miso_sw
    );

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
    variable idx: integer;-- helper
  begin
    if rising_edge(wb_clk_i) then
      if wb_rst_i='1' then
        framedatacount<=0;
        state <= idle;
        linkup <= '0';
        rxdatavalid <= '0';
        writeinprogress<='0';
        rxindex<=0;
        currentoffset<=(others => '0');
        ack_i <= '0';
        ien<='0';
        rxe<='0';
        int<='0';
        rxf<='0';
        txf<='0';
      else

        mi_wb_cyc_o<='0';
        mi_wb_stb_o<='0';
        mi_wb_we_o<=DontCareValue;
        ack_i <= '0';

        if wb_cyc_i='1' and wb_stb_i='1' and ack_i='0' then
          if wb_we_i='1' then
            if wb_adr_i(6 downto 5) = "00" then
              case wb_adr_i(4 downto 2) is
                when "000" =>
                  ien <= wb_dat_i(0);
                  rxe <= wb_dat_i(1);
                when "001" =>
                  if wb_dat_i(0)='1' then
                    int <= '0';
                    rxf<='0';
                  end if;
                  if wb_dat_i(1)='1' then
                    int <= '0';
                    txf<='1';
                  end if;
                when others =>
              end case;
            elsif wb_adr_i(6 downto 5) = "01" then
              idx := to_integer(unsigned(wb_adr_i(4 downto 2)));
              rxaddr(idx) <= wb_dat_i(maxAddrBitIncIO downto 8);
            end if;
          end if;
          wb_dat_o <= (others => 'X');
          case wb_adr_i(4 downto 2) is
            when "000" =>
              wb_dat_o(0) <= ien;
              wb_dat_o(1) <= rxe;
            when "001" =>
              wb_dat_o(0) <= rxf;
              wb_dat_o(1) <= txf;
            when "010" =>
              wb_dat_o <= std_logic_vector(to_unsigned(irxindex,32));

            when others =>
          end case;
          ack_i<='1';
        end if;

        -- State processing

        case state is

          when idle =>
            if frame='1' then -- Signalled a frame.
              state <= framecontrol;
            end if;

          when framecontrol =>
            if frame='1' then -- Hmm, an empty frame it seems ?
              linkup<='1';
            else
              if datavalid='1' then
                if linkup='1' then
                  state <= framedata;
                  rxdata(31 downto 8)<=rxdata(23 downto 0);
                  rxdata(7 downto 0) <= ddataout;
                  datacount <= 2;
                else
                  --state <= framedata;
                end if;
              end if;
            end if;

            if clkdetected='0' then
              -- We lost clock.
              linkup <= '0';
              state <= idle;
            end if;

          when framedata =>
            if datavalid='1' then
              --report "Data" severity failure;
              rxdata(31 downto 8)<=rxdata(23 downto 0);
              rxdata(7 downto 0) <= ddataout;
              if datacount=0 then
                datacount <= 3;
                state <= queuewrite;
              else
                datacount <= datacount - 1;
              end if;
            else
              -- If we see a frame, we are done.
              if frame='1' then
                state <= idle;
                -- notify, interrupt, increment rx buffer...
                rxindex <= rxindex+1;
                irxindex <= rxindex;
                rxf<='1';
                if ien='1' then
                  -- send interrupt to host...
                  int <='1';
                end if;
              end if;
            end if;
          when queuewrite =>
            rxdata_q <= rxdata;
            rxdatavalid<='1';
            state <= framedata;
          when others =>

        end case;

        -- Note: once we change the ACK method on SDRAM,
        -- we should not wait for ACK here.

        if rxdatavalid='1' then
          mi_wb_adr_o <= rxaddr(rxindex) & currentoffset;
          mi_wb_dat_o <= rxdata_q;
          mi_wb_cyc_o <= '1';
          mi_wb_stb_o <= '1';
          mi_wb_we_o <= '1';
          writeinprogress<='1';
        end if;

        if writeinprogress='1' then
          mi_wb_cyc_o<='1';
        end if;

        if mi_wb_stall_i='0' and writeinprogress='1' then
          --writeinprogress<='0';
          rxdatavalid<='0';
          --mi_wb_cyc_o<='0';
          mi_wb_stb_o<='0';
          mi_wb_we_o<=DontCareValue;
          mi_wb_adr_o <= (others => DontCareValue);
          mi_wb_dat_o <= (others => DontCareValue);
        end if;

        if writeinprogress='1' and mi_wb_ack_i='1' then
          mi_wb_cyc_o<='0';
          writeinprogress<='0';
          rxdatavalid<='0';
          -- Increment local pointer
          currentoffset <= std_logic_vector(unsigned(currentoffset)+4);
        end if;

      end if; -- wb_rst_i
    end if; -- rising_edge(wb_clk_i)
  end process;

  wb_ack_o <= ack_i;

end architecture;
