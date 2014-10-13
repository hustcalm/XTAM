// Copyright 2008 Isis Innovation Limited
// This is the main extry point for PTAM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"


using namespace std;
using namespace GVars3;

int main()
{
  cout << "  Welcome to XTAM " << endl;
  cout << "  --------------- " << endl;
  cout << "  Based on Parallel tracking and mapping for Small AR workspaces" << endl;
  cout << "  And Dense Tracking and Mapping" << endl;
  cout << "  Copyleft (C) Lihang Li@NLPR 2014 " << endl;  
  cout << endl;
  cout << "  Parsing settings.cfg ...." << endl;

  // Note that GUI is a global object living in GVars3
  // and comes to life once PTAM starts doing parser job in a dedicated thread
  GUI.LoadFile("settings.cfg");
 
  // Start parsing of the console input
  GUI.StartParserThread(); 

  // Call StopParserThread when PTAM terminates normally
  atexit(GUI.StopParserThread); 
  
  try
    {
      System s; // Construct the system using the default constructor
      s.Run();  // Everything happens here
    }
  catch(CVD::Exceptions::All e)
    {
      cout << endl;
      cout << "!! Failed to run system; got exception. " << endl;
      cout << "   Exception was: " << endl;
      cout << e.what << endl;
    }
}

