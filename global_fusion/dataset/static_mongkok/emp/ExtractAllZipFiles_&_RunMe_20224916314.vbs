' "ExtractAllZipFiles_&_RunMe_20224916314.vbs" is a vbscript program,
' it helps you to extract all RINEX files into a single folder.
' Please copy all ZIP files together with this vbscript program to a folder,
' and then double-click "ExtractAllZipFiles_&_RunMe_20224916314.vbs" to run.
' It will extract all RINEX files from individual ZIP files to a single subfolder "Output"

Dim zip(0)
zip(0)="r3_30s_24h_hksc_2019_06_10.zip"

Count=0

CurrentDirectory = left(WScript.ScriptFullName,(Len(WScript.ScriptFullName))-(len(WScript.ScriptName)))
ExtractToTemp = currentDirectory & "Temp_" & year(Now()) & month(Now()) & day(Now()) & hour(Now()) & minute(Now()) & second(Now())
OutputSubFolderName="Output"
OutputFolder= currentDirectory & OutputSubFolderName & "\"

Set fso = CreateObject("Scripting.FileSystemObject")

If NOT fso.FolderExists(OutputFolder) Then
   fso.CreateFolder(OutputFolder)
End If


for i= 0 to ubound(zip)
   if zip(i)<>"" then
      If NOT fso.FolderExists(ExtractToTemp) Then
         fso.CreateFolder(ExtractToTemp)
      End If

      ExtractOneZip zip(i)

      MoveAllFilesInFolder ExtractToTemp
      fso.DeleteFolder ExtractToTemp
   end if
next


Set fso = Nothing

msgbox trim(Count) & " files extracted in subfolder: " & OutputSubFolderName, 64, "Download RINEX File from SatRef"

WScript.quit


'-------------------------------------------------
Sub ExtractOneZip(ZipFile)

  ZipFile = currentDirectory & ZipFile

  set objShell = CreateObject("Shell.Application")
  set FilesInZip=objShell.NameSpace(ZipFile).items
  objShell.NameSpace(ExtractToTemp).CopyHere(FilesInZip)
  Set objShell = Nothing

end sub

'-------------------------------------------------
Sub MoveAllFilesInFolder(folderspec)

  Set folder = fso.GetFolder(folderspec)
  Set fc = folder.SubFolders

  MoveAllFilesinTheFolder folderspec 

  For Each f1 in fc
    
    MoveAllFilesInFolder folderspec & "\" & f1.name
    MoveAllFilesinTheFolder folderspec & "\" & f1.name
        
  Next

End Sub

'-------------------------------------------------
Private Sub MoveAllFilesinTheFolder (sFolder)

  Set afolder = fso.GetFolder(sFolder)
  Set files = afolder.Files    

  For each file In files    

    if fso.FileExists(OutputFolder & file.Name) then
       fso.DeleteFile(OutputFolder & file.Name) 
    end if
       
    fso.MoveFile sFolder & "\" & file.Name, OutputFolder
    Count = Count +1

  Next  

end sub
