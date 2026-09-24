// Selectively disassemble/decompile functions from a supplied address list.
// @category WarThunder
import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.*;
import ghidra.app.cmd.disassemble.DisassembleCommand;
import ghidra.program.model.address.*;
import ghidra.program.model.listing.*;
import java.nio.file.*;
import java.util.*;

public class ExportFM extends GhidraScript {
    public void run() throws Exception {
        String[] args = getScriptArgs();
        List<String> entries = Files.readAllLines(Path.of(args[0]));
        Path out = Path.of(args[1]);
        Files.createDirectories(out);
        ArrayList<Function> funcs = new ArrayList<>();
        for (String line : entries) {
            if (line.isBlank() || line.startsWith("#")) continue;
            String[] parts = line.trim().split("\\s+");
            Address a = toAddr(Long.parseUnsignedLong(parts[0].replace("0x", ""),16));
            Address end = toAddr(Long.parseUnsignedLong(parts[1].replace("0x", ""),16)-1);
            AddressSet body = new AddressSet(a,end);
            new DisassembleCommand(a,body,true).applyTo(currentProgram,monitor);
            Function f = getFunctionAt(a);
            if (f == null) f = createFunction(a, parts.length>2?parts[2]:null);
            if (f != null) {
                if(parts.length>2) f.setName(parts[2],ghidra.program.model.symbol.SourceType.USER_DEFINED);
                funcs.add(f);
            } else println("FAILED_CREATE "+a);
        }
        DecompInterface dec = new DecompInterface();
        DecompileOptions opts = new DecompileOptions();
        opts.grabFromProgram(currentProgram);
        dec.setOptions(opts);
        dec.openProgram(currentProgram);
        for(Function f:funcs) {
            DecompileResults result = dec.decompileFunction(f,90,monitor);
            String name=f.getEntryPoint().toString();
            if(result.decompileCompleted()) {
                Files.writeString(out.resolve(name+".c"),"// Unmodified Ghidra output; inferred types and names require verification.\n// Unslid entry: "+name+"\n"+result.getDecompiledFunction().getC());
                println("EXPORTED "+name+" "+f.getBody().getNumAddresses());
            } else println("FAILED_DECOMPILE "+name+" "+result.getErrorMessage());
        }
        dec.dispose();
    }
}
