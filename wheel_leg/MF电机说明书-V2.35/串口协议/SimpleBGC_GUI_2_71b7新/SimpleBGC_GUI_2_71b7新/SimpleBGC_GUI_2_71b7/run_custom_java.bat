@SET JAVA_EXE="C:\Program Files (x86)\Java\jre1.8.0_321\bin\javaw.exe"

%JAVA_EXE% -Dsimplebgc_gui.SimpleBGC_GUIView.Logger.level=0 -Djava.library.path="lib" -Dlog4j.configuration=log4j.properties -Dgnu.io.rxtx.NoVersionOutput=true -Dsun.java2d.dpiaware=false  -jar SimpleBGC_GUI.jar 


