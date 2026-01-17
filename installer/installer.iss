#ifndef SourceDir
  #error SourceDir not defined. Use /DSourceDir="C:\path\to\portable\folder"
#endif
#ifndef NiInstaller
  #error NiInstaller not defined. Use /DNiInstaller="C:\path\to\ni-daqmx_installer.exe"
#endif
#ifndef VcRedist
  #error VcRedist not defined. Use /DVcRedist="C:\path\to\vcredist_x64.exe"
#endif

#define NiInstallerFile ExtractFileName(NiInstaller)
#define VcRedistFile ExtractFileName(VcRedist)

#define AppName "Visual Droplet Sorting"
#define AppVersion "1.0.0"
#define AppPublisher "haeminjung"
#define AppExeName "droplet_pipeline.exe"
#define DefaultDirName "{pf}\VisualDropletSorting"
#ifndef OutputDir
  #define OutputDir SourcePath + "\output"
#endif

[Setup]
AppId={{0A2C6D54-74A9-4A7C-9B55-0878E1A5CE9B}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher={#AppPublisher}
DefaultDirName={#DefaultDirName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
OutputDir={#OutputDir}
OutputBaseFilename=VisualDropletSortingSetup
Compression=lzma
SolidCompression=yes
ArchitecturesInstallIn64BitMode=x64
PrivilegesRequired=admin
UninstallDisplayIcon={app}\{#AppExeName}
InfoBeforeFile=preinstall_note.txt

[Files]
Source: "{#SourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#NiInstaller}"; DestDir: "{tmp}"; Flags: deleteafterinstall
Source: "{#VcRedist}"; DestDir: "{tmp}"; Flags: deleteafterinstall

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExeName}"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"

[Run]
Filename: "{tmp}\{#NiInstallerFile}"; StatusMsg: "Installing NI-DAQmx..."; Flags: postinstall waituntilterminated
Filename: "{tmp}\{#VcRedistFile}"; StatusMsg: "Installing VC++ Redistributable..."; Flags: postinstall waituntilterminated
