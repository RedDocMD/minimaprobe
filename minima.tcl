adapter serial E6614104037B2432
# adapter serial E661801017056335
adapter driver cmsis-dap
transport select swd
adapter speed 10000

if [catch {transport select}] {
  echo "Error: unable to select a session transport. Can't continue."
  shutdown
}

proc swj_newdap {chip tag args} {
 if [using_jtag] {
     eval jtag newtap $chip $tag $args
 } elseif [using_swd] {
     eval swd newdap $chip $tag $args
 } else {
     echo "Error: transport '[ transport select ]' not supported by swj_newdap"
     shutdown
 }
}

swj_newdap minima cpu -irlen 4
dap create minima.dap0 -chain-position minima.cpu
target create minima.core0 cortex_m -dap minima.dap0 -coreid 0
minima.core0 cortex_m reset_config sysresetreq

