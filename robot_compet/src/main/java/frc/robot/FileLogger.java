package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class FileLogger {
  private final File file;
  private final BufferedWriter bufferedWriter;

  public FileLogger(String filename, boolean append) {
    try {
      FileWriter fileWriter = null;

      File folder = new File("/tmp/robotlogs");
      folder.mkdirs();

      file = new File(folder, filename);
      if (!file.exists()) {
        file.createNewFile();
      }
      fileWriter = new FileWriter(file, append);

      bufferedWriter = new BufferedWriter(fileWriter);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public void writeText(String text) {
    try {
      bufferedWriter.write(text);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public void close() {
    try {
      bufferedWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

}
