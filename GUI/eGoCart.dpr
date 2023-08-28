program eGoCart;

uses
  Forms,
  mainfrm in 'mainfrm.pas' {mainform};

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(Tmainform, mainform);
  Application.Run;
end.
