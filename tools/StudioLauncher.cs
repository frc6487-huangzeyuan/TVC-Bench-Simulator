using System;
using System.Diagnostics;
using System.IO;
using System.Windows.Forms;
[assembly:System.Reflection.AssemblyTitle("TVC Studio")]
[assembly:System.Reflection.AssemblyProduct("TVC Studio")]
[assembly:System.Reflection.AssemblyVersion("0.1.0.0")]
[assembly:System.Reflection.AssemblyFileVersion("0.1.0.0")]
static class StudioLauncher {
  [STAThread] static void Main() {
    string root=AppDomain.CurrentDomain.BaseDirectory;
    string python=Path.Combine(root,@".venv\Scripts\pythonw.exe");
    if(!File.Exists(python)) {
      MessageBox.Show("未找到本项目的 Python 环境。请保持 TVC Studio.exe 与 .venv 文件夹位于同一目录。","TVC Studio",MessageBoxButtons.OK,MessageBoxIcon.Information);return;
    }
    try {
      var info=new ProcessStartInfo(python,"\""+Path.Combine(root,"run_studio.py")+"\"");
      info.WorkingDirectory=root;info.UseShellExecute=false;info.CreateNoWindow=true;info.WindowStyle=ProcessWindowStyle.Hidden;
      Process.Start(info);
    } catch(Exception e) { MessageBox.Show("仿真环境启动失败："+e.Message,"TVC Studio",MessageBoxButtons.OK,MessageBoxIcon.Error); }
  }
}
