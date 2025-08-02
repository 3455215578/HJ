#!/bin/sh
java -Dsimplebgc_gui.SimpleBGC_GUIView.Logger.level=0 -Djava.library.path="lib" -Dlog4j.configuration=log4j.properties -Dgnu.io.rxtx.NoVersionOutput=true -Djava.library.path=./lib -Djava.library.path=./lib/natives -Dsun.java2d.dpiaware=false -jar SimpleBGC_GUI.jar

# append DEBUG_MODE to the end enable some debugging functions