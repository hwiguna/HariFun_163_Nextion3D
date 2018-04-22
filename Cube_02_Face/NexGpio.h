// TODO: Pull request this to NeoNextion.

class NexGpio {
  public:
    NexGpio()
    {
      //nex = nextion;
    };

   void pin_mode(Nextion nex, uint32_t port, uint32_t mode, uint32_t control_id)
    {
      char buf;
      String cmd;

      cmd += "cfgpio ";
      buf = port + '0';
      cmd += buf;
      cmd += ',';
      buf = mode + '0';
      cmd += buf;
      cmd += ',';
      buf = control_id = '0';
      cmd += buf;

      nex.sendCommand(cmd.c_str());
    };

    uint16_t getGpio(Nextion nex, int port)
    {
      String cmd = "get pio";
      char buf = port + '0';
      cmd += buf;

      nex.sendCommand(cmd.c_str());
      uint32_t val;
      if (nex.receiveNumber(&val))
        return val;
      else
        return 0;
    };
// private:
// Nextion nex;
};

