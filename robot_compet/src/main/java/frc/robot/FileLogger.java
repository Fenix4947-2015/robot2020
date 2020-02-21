package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class FileLogger {
  private final File file;
  private final BufferedWriter bufferedWriter;

  public FileLogger(String filename) throws IOException {
    FileWriter fileWriter = null;

    file = new File("/home/lvuser/" + filename);
    if (!file.exists()) {
      file.createNewFile();
    }
    fileWriter = new FileWriter(file);

    bufferedWriter = new BufferedWriter(fileWriter);
  }

  public void writeText(String text) throws IOException {
    bufferedWriter.write(text);
  }

  public void close() throws IOException {
    bufferedWriter.close();
  }

}
