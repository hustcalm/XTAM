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
  cout << "  Welcome to PTAM " << endl;
  cout << "  --------------- " << endl;
  cout << "  Parallel tracking and mapping for Small AR workspaces" << endl;
  cout << "  Copyright (C) Isis Innovation Limited 2008 " << endl;  
  cout << endl;
  cout << "  Parsing settings.cfg ...." << endl;

  // Note that GUI is a global object living in GVars3
  // and comes to life once PTAM starts
  // doing parser job in a dedicated thread
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

