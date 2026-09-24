// Export the unmodified input bytes retained in the analysis database.
// @category WarThunder
import ghidra.app.script.GhidraScript;
import ghidra.program.database.mem.FileBytes;
import java.nio.file.*;
import java.io.*;

public class ExportOriginalBinary extends GhidraScript {
    public void run() throws Exception {
        var files = currentProgram.getMemory().getAllFileBytes();
        if (files.size() != 1) throw new IOException("Expected one original input file");
        FileBytes file = files.get(0);
        try (OutputStream out = Files.newOutputStream(Path.of(getScriptArgs()[0]))) {
            byte[] buffer = new byte[1048576];
            for (long offset = 0; offset < file.getSize();) {
                monitor.checkCancelled();
                int count = (int)Math.min(buffer.length, file.getSize() - offset);
                int read = file.getOriginalBytes(offset, buffer, 0, count);
                if (read != count) throw new IOException("Incomplete original bytes");
                out.write(buffer, 0, read);
                offset += read;
            }
        }
        println("Exported " + file.getSize() + " original bytes");
    }
}
