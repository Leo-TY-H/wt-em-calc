// Apply signatures recovered by matching published Polares layout to instructions.
// @category WarThunder
import ghidra.app.script.GhidraScript;
import ghidra.program.model.data.*;
import ghidra.program.model.listing.*;
import ghidra.program.model.symbol.SourceType;

public class TypePolars extends GhidraScript {
    void signature(long address,String name,DataType ret,String[] names,DataType... types) throws Exception {
        Function f=getFunctionAt(toAddr(address));
        if(f==null){ disassemble(toAddr(address)); f=createFunction(toAddr(address),name); }
        f.setName(name,SourceType.USER_DEFINED);
        f.setReturnType(ret,SourceType.USER_DEFINED);
        Parameter[] ps=new Parameter[types.length];
        for(int i=0;i<types.length;i++)ps[i]=new ParameterImpl(names[i],types[i],currentProgram);
        f.replaceParameters(Function.FunctionUpdateType.DYNAMIC_STORAGE_ALL_PARAMS,true,SourceType.USER_DEFINED,ps);
    }
    public void run() throws Exception {
        DataType fl=FloatDataType.dataType, vo=VoidDataType.dataType;
        FunctionIterator iterator=currentProgram.getFunctionManager().getFunctions(true);
        while(iterator.hasNext()) {
            Function f=iterator.next();String n=f.getName().replaceFirst("^_+", "");
            if(n.equals("stack_chk_fail") || n.equals("abort"))f.setNoReturn(true);
            if(n.matches("(sin|cos|tan|asin|acos|atan|atan2|exp|pow|fmod|sqrt|log)f?")) {
                DataType scalar=n.endsWith("f")?fl:DoubleDataType.dataType;
                int count=n.matches("(atan2|pow|fmod)f?")?2:1;
                f.setReturnType(scalar,SourceType.USER_DEFINED);
                Parameter[] ps=new Parameter[count];
                for(int i=0;i<count;i++)ps[i]=new ParameterImpl("arg"+i,scalar,currentProgram);
                f.replaceParameters(Function.FunctionUpdateType.DYNAMIC_STORAGE_ALL_PARAMS,true,SourceType.USER_DEFINED,ps);
            }
        }
        Function stackFail=getFunctionAt(toAddr(0x106e613edL));
        if(stackFail!=null)stackFail.setNoReturn(true);
        StructureDataType polar=new StructureDataType("Polares",0);
        for(String name:new String[]{"cl0","cd0","indCoeff","clLineCoeff","cyCritH","cyCritL","aoaCritH","aoaCritL","aoaLineH","aoaLineL","parabCyCoeffH","parabCyCoeffL","aerCenterOffset","clToCm0","clToCm1","parabAngle","declineCoeff","maxDistAng","cdAfterCoeff","clAfterCritL","clAfterCritH","kq","clKq","cyMult"})polar.add(fl,name,null);
        DataType dt=currentProgram.getDataTypeManager().addDataType(polar,DataTypeConflictHandler.REPLACE_HANDLER);
        DataType ptr=new PointerDataType(dt), opaque=new PointerDataType(vo), fptr=new PointerDataType(fl);
        StructureDataType p2=new StructureDataType("Point2",0);p2.add(fl,"x",null);p2.add(fl,"y",null);
        DataType point2=currentProgram.getDataTypeManager().addDataType(p2,DataTypeConflictHandler.REPLACE_HANDLER);
        signature(0x10198c320L,"polar_calc_c",point2,new String[]{"polar","aoa","angle","cl_add","cd_coeff"},ptr,fl,fl,fl,fl);
        signature(0x10198c440L,"polar_calc_cd",fl,new String[]{"polar","aoa"},ptr,fl);
        signature(0x10198c4d0L,"polar_calc_cl",fl,new String[]{"polar","aoa"},ptr,fl);
        signature(0x10198eab0L,"polar_calc_mach",vo,new String[]{"props","mach","out"},opaque,fl,ptr);
        signature(0x10198f3d0L,"polar_calc_mach_cy_mult",vo,new String[]{"props","cy_mult","mach","out"},opaque,fl,fl,ptr);
        signature(0x10198eb10L,"polar_calc_mach_dependent",vo,new String[]{"props","cl","clCritH","clCritL","aoaCritH","aoaCritL","cd","mach","out"},opaque,fl,fl,fl,fl,fl,fl,fl,ptr);
        signature(0x10198c7f0L,"polar_solve_aoa",BooleanDataType.dataType,new String[]{"polar","cy","out_aoa"},ptr,fl,fptr);
        signature(0x10198c900L,"polar_solve_aoa_rotated",BooleanDataType.dataType,new String[]{"polar","cy","ang","convert_aoa","out_aoa"},ptr,fl,fl,BooleanDataType.dataType,fptr);
    }
}
