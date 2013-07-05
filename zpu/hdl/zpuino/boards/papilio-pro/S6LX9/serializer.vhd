library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
-- synthesis translate_off
use work.txt_util.all;
-- synthesis translate_on

entity serializer is
  port (
    clk: in std_logic;
    rst: in std_logic;
    clkout: out std_logic;

    sdata:  out std_logic;
    -- Interface for FIFO
    data_in: in std_logic_vector(32 downto 0);
    data_read: out std_logic;
    data_avail: in std_logic
  );
end entity serializer;

architecture sim of serializer is

  signal shreg: std_logic_vector(31 downto 0);
  signal txdata: std_logic_vector(31 downto 0);
  signal cnt: integer range 0 to 31;
  signal bscount: std_logic_vector(5 downto 0);

  type state_type is (
    startframe,
    idledata,
    shift
  );

  signal state: state_type;
  signal stuff: std_logic;
  signal ferr: std_logic;
  signal has_data: std_logic;
  signal fid: std_logic;

begin

  clkout <= clk;

  process(clk)
  begin
    if falling_edge(clk) then
      if bscount(5)='1' then
        sdata<='0';
      else
        sdata <= shreg(shreg'HIGH);
      end if;
    end if;
  end process;


  process(clk)
  begin
    if rising_edge(clk) then
      if rst='1' then
        state <= startframe;
        stuff <= '0';
        has_data <= '0';
        data_read<='0';
        shreg(shreg'HIGH downto shreg'HIGH-7) <= "01111111";
        cnt <= 7;
        bscount <= "000001";
        fid <= '0';
      else
        data_read <= '0';

        case state is

          when startframe =>

            if cnt=0 then

              if data_avail='1' then  -- We have data, shift it
                if data_in(32)='1' then
                  state <= shift;
                  bscount<= "000001";
                  shreg <= data_in(31 downto 0);
                  fid <= '0';--data_in(32);
                  cnt <= 31;
                else
                  -- signal error here.
                end if;
                -- Ack read
                data_read <= '1';
              else
                shreg<=(others => 'X');
                shreg(shreg'HIGH downto shreg'HIGH-7) <= "01111111";
                cnt <= 7;
              end if;
            else
              cnt <= cnt - 1;
              shreg(shreg'HIGH downto 1)<=shreg(shreg'HIGH-1 downto 0);
              shreg(0)<='X';
            end if;

          when idledata =>

            if cnt=0 then

              if data_avail='1' then  -- We have data, shift it
                state <= shift;
                bscount<="000001";
                shreg <= data_in(31 downto 0);
                fid <= data_in(32);
                cnt <= 31;
                -- Ack read
                data_read <= '1';
              else
                shreg<=(others => 'X');
                shreg(shreg'HIGH downto shreg'HIGH-7) <= "01111110";
                cnt <= 7;
              end if;
            else
              cnt <= cnt - 1;
              shreg(shreg'HIGH downto 1)<=shreg(shreg'HIGH-1 downto 0);
              shreg(0)<='X';--shreg(shreg'HIGH);
            end if;

          when shift =>
              if cnt=0 then
                if bscount(5)='0' then
                  if fid='1' then
                    -- Last data for a frame.
                    state <= startframe;
                    shreg<=(others => 'X');
                    shreg(shreg'HIGH downto shreg'HIGH-7) <= "01111111";
                    cnt <= 7;
                  else
                    if data_avail='1' then  -- We have more data, shift it
                      --state <= shift;
                      bscount<="000001";
                      shreg <= data_in(31 downto 0);
                      cnt <= 31;
                      fid <= data_in(32);
                      -- Ack read
                      data_read <= '1';
                    
                      --state <= startframe;
                    else
                      -- No data, we need to transmit an idle tag
                      shreg<=(others => 'X');
                      shreg(shreg'HIGH downto shreg'HIGH-7) <= "01111110";
                      cnt<=7;
                      state <= idledata;
                    end if;
                  end if;
                end if;
              else
                if bscount(5)='0' then
                  cnt <= cnt - 1;
                end if;

                -- Check for bit stuff.
                if shreg(shreg'HIGH)='1' then
                  if bscount(5)='0' then
                    bscount(5 downto 1) <= bscount(4 downto 0);
                    bscount(0)<='1';
                  end if;
                else
                  bscount <= "000001";
                end if;
  
                if bscount(5)='1' then
                  bscount <= "000001";
                else
                  shreg(shreg'HIGH downto 1)<=shreg(shreg'HIGH-1 downto 0);
                  shreg(0)<='X';
                end if;
              end if;
           when others => null;
        end case;
      end if;
    end if;
  end process;

end sim;
