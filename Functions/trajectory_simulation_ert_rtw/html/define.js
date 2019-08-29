function CodeDefine() { 
this.def = new Array();
this.def["rt_OneStep"] = {file: "ert_main_c.html",line:39,type:"fcn"};this.def["main"] = {file: "ert_main_c.html",line:89,type:"fcn"};this.def["rtX"] = {file: "trajectory_simulation_c.html",line:47,type:"var"};this.def["rtDW"] = {file: "trajectory_simulation_c.html",line:50,type:"var"};this.def["rtM_"] = {file: "trajectory_simulation_c.html",line:53,type:"var"};this.def["rtM"] = {file: "trajectory_simulation_c.html",line:54,type:"var"};this.def["BigEndianIEEEDouble"] = {file: "trajectory_simulation_c.html",line:114,type:"type"};this.def["LittleEndianIEEEDouble"] = {file: "trajectory_simulation_c.html",line:121,type:"type"};this.def["IEEESingle"] = {file: "trajectory_simulation_c.html",line:128,type:"type"};this.def["rtInf"] = {file: "trajectory_simulation_c.html",line:130,type:"var"};this.def["rtMinusInf"] = {file: "trajectory_simulation_c.html",line:131,type:"var"};this.def["rtNaN"] = {file: "trajectory_simulation_c.html",line:132,type:"var"};this.def["rtInfF"] = {file: "trajectory_simulation_c.html",line:133,type:"var"};this.def["rtMinusInfF"] = {file: "trajectory_simulation_c.html",line:134,type:"var"};this.def["rtNaNF"] = {file: "trajectory_simulation_c.html",line:135,type:"var"};this.def["rtGetInf"] = {file: "trajectory_simulation_c.html",line:141,type:"fcn"};this.def["rtGetInfF"] = {file: "trajectory_simulation_c.html",line:165,type:"fcn"};this.def["rtGetMinusInf"] = {file: "trajectory_simulation_c.html",line:176,type:"fcn"};this.def["rtGetMinusInfF"] = {file: "trajectory_simulation_c.html",line:200,type:"fcn"};this.def["rtGetNaN"] = {file: "trajectory_simulation_c.html",line:211,type:"fcn"};this.def["rtGetNaNF"] = {file: "trajectory_simulation_c.html",line:235,type:"fcn"};this.def["rt_InitInfAndNaN"] = {file: "trajectory_simulation_c.html",line:247,type:"fcn"};this.def["rtIsInf"] = {file: "trajectory_simulation_c.html",line:259,type:"fcn"};this.def["rtIsInfF"] = {file: "trajectory_simulation_c.html",line:265,type:"fcn"};this.def["rtIsNaN"] = {file: "trajectory_simulation_c.html",line:271,type:"fcn"};this.def["rtIsNaNF"] = {file: "trajectory_simulation_c.html",line:293,type:"fcn"};this.def["rt_ertODEUpdateContinuousStates"] = {file: "trajectory_simulation_c.html",line:305,type:"fcn"};this.def["rt_atan2d_snf"] = {file: "trajectory_simulation_c.html",line:376,type:"fcn"};this.def["rt_powd_snf"] = {file: "trajectory_simulation_c.html",line:412,type:"fcn"};this.def["trajectory_simulation_step"] = {file: "trajectory_simulation_c.html",line:459,type:"fcn"};this.def["trajectory_simulation_derivatives"] = {file: "trajectory_simulation_c.html",line:614,type:"fcn"};this.def["trajectory_simulation_initializer"] = {file: "trajectory_simulation_c.html",line:630,type:"fcn"};this.def["RT_MODEL"] = {file: "trajectory_simulation_h.html",line:64,type:"type"};this.def["DW"] = {file: "trajectory_simulation_h.html",line:79,type:"type"};this.def["X"] = {file: "trajectory_simulation_h.html",line:84,type:"type"};this.def["XDot"] = {file: "trajectory_simulation_h.html",line:89,type:"type"};this.def["XDis"] = {file: "trajectory_simulation_h.html",line:94,type:"type"};this.def["ODE4_IntgData"] = {file: "trajectory_simulation_h.html",line:103,type:"type"};this.def["int8_T"] = {file: "rtwtypes_h.html",line:53,type:"type"};this.def["uint8_T"] = {file: "rtwtypes_h.html",line:54,type:"type"};this.def["int16_T"] = {file: "rtwtypes_h.html",line:55,type:"type"};this.def["uint16_T"] = {file: "rtwtypes_h.html",line:56,type:"type"};this.def["int32_T"] = {file: "rtwtypes_h.html",line:57,type:"type"};this.def["uint32_T"] = {file: "rtwtypes_h.html",line:58,type:"type"};this.def["int64_T"] = {file: "rtwtypes_h.html",line:59,type:"type"};this.def["uint64_T"] = {file: "rtwtypes_h.html",line:60,type:"type"};this.def["real32_T"] = {file: "rtwtypes_h.html",line:61,type:"type"};this.def["real64_T"] = {file: "rtwtypes_h.html",line:62,type:"type"};this.def["real_T"] = {file: "rtwtypes_h.html",line:68,type:"type"};this.def["time_T"] = {file: "rtwtypes_h.html",line:69,type:"type"};this.def["boolean_T"] = {file: "rtwtypes_h.html",line:70,type:"type"};this.def["int_T"] = {file: "rtwtypes_h.html",line:71,type:"type"};this.def["uint_T"] = {file: "rtwtypes_h.html",line:72,type:"type"};this.def["ulong_T"] = {file: "rtwtypes_h.html",line:73,type:"type"};this.def["ulonglong_T"] = {file: "rtwtypes_h.html",line:74,type:"type"};this.def["char_T"] = {file: "rtwtypes_h.html",line:75,type:"type"};this.def["uchar_T"] = {file: "rtwtypes_h.html",line:76,type:"type"};this.def["byte_T"] = {file: "rtwtypes_h.html",line:77,type:"type"};this.def["pointer_T"] = {file: "rtwtypes_h.html",line:98,type:"type"};}
CodeDefine.instance = new CodeDefine();
var testHarnessInfo = {OwnerFileName: "", HarnessOwner: "", HarnessName: "", IsTestHarness: "0"};
var relPathToBuildDir = "../ert_main.c";
var fileSep = "\\";
var isPC = true;
function Html2SrcLink() {
	this.html2SrcPath = new Array;
	this.html2Root = new Array;
	this.html2SrcPath["ert_main_c.html"] = "../ert_main.c";
	this.html2Root["ert_main_c.html"] = "ert_main_c.html";
	this.html2SrcPath["trajectory_simulation_c.html"] = "../trajectory_simulation.c";
	this.html2Root["trajectory_simulation_c.html"] = "trajectory_simulation_c.html";
	this.html2SrcPath["trajectory_simulation_h.html"] = "../trajectory_simulation.h";
	this.html2Root["trajectory_simulation_h.html"] = "trajectory_simulation_h.html";
	this.html2SrcPath["rtwtypes_h.html"] = "../rtwtypes.h";
	this.html2Root["rtwtypes_h.html"] = "rtwtypes_h.html";
	this.getLink2Src = function (htmlFileName) {
		 if (this.html2SrcPath[htmlFileName])
			 return this.html2SrcPath[htmlFileName];
		 else
			 return null;
	}
	this.getLinkFromRoot = function (htmlFileName) {
		 if (this.html2Root[htmlFileName])
			 return this.html2Root[htmlFileName];
		 else
			 return null;
	}
}
Html2SrcLink.instance = new Html2SrcLink();
var fileList = [
"ert_main_c.html","trajectory_simulation_c.html","trajectory_simulation_h.html","rtwtypes_h.html"];
