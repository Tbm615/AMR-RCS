; Windows installer for RCS (Inno Setup 6). CI runs:
;   ISCC.exe /DAppVersion=0.6.1 installer\rcs.iss
; from the windowed PyInstaller build in dist\rcs. Per-user by default (no
; admin needed), Start menu entry, optional desktop icon, uninstaller.
#ifndef AppVersion
  #define AppVersion "0.0.0"
#endif

[Setup]
AppId={{6F3E2C1B-8C1A-4E7B-9C55-3D2A7F0B9E41}
AppName=RCS
AppVersion={#AppVersion}
AppVerName=RCS {#AppVersion}
AppPublisher=b4lanc3
AppPublisherURL=https://github.com/Tbm615/AMR-RCS
DefaultDirName={autopf}\RCS
DefaultGroupName=RCS
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
OutputDir=..\dist
OutputBaseFilename=rcs-windows-x86_64-setup
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
ArchitecturesInstallIn64BitMode=x64compatible
UninstallDisplayName=RCS {#AppVersion}

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "..\dist\rcs\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\RCS"; Filename: "{app}\rcs.exe"
Name: "{group}\Uninstall RCS"; Filename: "{uninstallexe}"
Name: "{autodesktop}\RCS"; Filename: "{app}\rcs.exe"; Tasks: desktopicon

[Run]
Filename: "{app}\rcs.exe"; Description: "{cm:LaunchProgram,RCS}"; Flags: nowait postinstall skipifsilent
