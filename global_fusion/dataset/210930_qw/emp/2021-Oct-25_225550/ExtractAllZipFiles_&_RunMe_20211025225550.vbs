' "ExtractAllZipFiles_&_RunMe_20211025225550.vbs"是一個VBScript程序，幫助您解壓縮所有RINEX文件到單一子文件夾。
' 首先請複製此VBScript程序及所有ZIP壓縮文件到一個文件夾，然後雙擊執行此VBScript程序。
' 它便會解壓縮所有Zip文件到子文件夾。

Dim zip(0)
zip(0)="r3_30s_24h_hksc_2021_09_30.zip"

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
