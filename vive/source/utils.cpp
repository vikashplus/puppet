
// Read Configuration variables ===============
#include <stdio.h>
#include <string>
#include <Windows.h>
#include "utils.h"

// Utilities ======================
cgOption option;

// write error message to console, pause and exit
void util_error(const char* msg)
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		HANDLE hStdoubt = GetStdHandle(STD_OUTPUT_HANDLE);
		SetConsoleTextAttribute(hStdoubt, FOREGROUND_INTENSITY | FOREGROUND_RED);
#endif
		printf("ERROR: %s\n\nPress Enter to exit ...", msg);
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		SetConsoleTextAttribute(hStdoubt, FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED);
#endif
		// pause, exit
		getchar();
		exit(1);
}

// write warning message to console
void util_warning(const char* msg)
{
		// write to console
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		HANDLE hStdoubt = GetStdHandle(STD_OUTPUT_HANDLE);
		SetConsoleTextAttribute(hStdoubt, FOREGROUND_INTENSITY | FOREGROUND_GREEN | FOREGROUND_RED);
#endif
		printf("\nWARNING: %s\n\n", msg);
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		SetConsoleTextAttribute(hStdoubt, FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_RED);
#endif
}

// allocate memory, align
void* util_malloc(unsigned int sz, unsigned int align)
{
	void* ptr = 0;

	// try to allocate
#if defined(__linux__) || defined(__APPLE__)
   // could use a check for if "sz" is a multiple of "align"
	if (align == 8 || sz % align) {
		ptr = malloc(sz); // malloc is always 8 aligned
	}
	else {
		posix_memalign(&ptr, align, sz);
	}
#else
	ptr = _aligned_malloc(sz, align);
#endif

	// error if null pointer
	if( !ptr ) {
		util_error("could not allocate memory");
   }

	return ptr;
}

// free memory
void util_free(void* ptr)
{
#if defined(__linux__) || defined(__APPLE__)
	free(ptr);
#else
	_aligned_free(ptr);
#endif
}

// Open file with specified path (else locally)
FILE* util_fopen(const char* fileName, const char* mode)
{
	FILE* fp;
	char errmsg[300];
	int i=0;
	fopen_s(&fp, fileName, mode);
	if(fp)
		return fp;
	else //strip out path
	{	
		for(i=(int)strlen(fileName)-1;i>=0;i--)
			if(fileName[i]=='\\')
			{	fileName=fileName+i+1;
				break;
			}
		fopen_s(&fp, fileName, mode);
	}
	if(fp)
		return fp;
	else
	{
		sprintf_s(errmsg, "Problem opening file '%s'", fileName );
		util_error(errmsg);
		return NULL;
	}
}

// Read cgdata from a tab(or space) seperated file
int util_readFile(const char* Fname, cgNum* vec, const int size)
{
	int n_read, i;
	FILE* fp;
	char errmsg[300];
	const int str_n = 5;
	char str[str_n];
	char fmt[100];

	 // open the file, check for errors
	fp = util_fopen(Fname, "r" );
	
   // Read lines for values
	for(i = 0; i<size; i++)
	{	n_read = fscanf_s(fp, "%lf,", &vec[i]);
		if (n_read!=1)
		{	sprintf_s(fmt, "%%%ds", str_n);
			n_read = fscanf_s(fp, fmt, str);
			for(int j = 0; str[j]; j++)
				str[j] = tolower(str[j]);
			if( strcmp(str, "nan")==0) // if is a NaN
				vec[i] = sqrt(-1.0);
			else
			{	sprintf_s(errmsg, "Problem reading file '%s'. Check cgdata format", Fname );
				util_error(errmsg);
			}
		}
	}
	fclose (fp);
	return 0;
}

// vector dot-product... FMA ???
cgNum util_dot(const cgNum* vec1, const cgNum* vec2, const int n)
{
	int i = 0;
	cgNum res = 0;

	for( i=0; i<n; i++ )
		res += vec1[i] * vec2[i];

	return res;	
}

// multiply matrix and vector
void util_mulMatVec(cgNum* res, const cgNum* mat, const cgNum* vec,
				   int nr, int nc)
{
	int r;
	for( r=0; r<nr; r++ )
	   res[r] = util_dot(mat + r*nc, vec, nc);
}
// check for white spaces
int iswhite(char a){return a==' '||a=='\t';}

// Read values
int util_config(const char *fileName, const char *iname, void *var)
{
	const int maxInputSz = 1024;
	int lineCnt=0,icnt=0,q1;
	int numw, err=-1, len;
	char line[maxInputSz],lineRaw[maxInputSz];
	char name[maxInputSz];
	char* wordPtr[16];
	char* input[2];
	FILE* file;
	char errmsg[300];

	// parse input for variable name
	strcpy_s(name, maxInputSz, iname);
	len = (int)strlen(name);
	for(q1=0; q1<len; q1++)
	{
		if((q1==0&&!iswhite(name[0]))||(name[q1-1]==0&&(!iswhite(name[q1]))))
			if(icnt<2)
				input[icnt++]=name+q1;
		if(iswhite(name[q1]))
			name[q1]=0;
	}
  
	// parse file for values
	file = util_fopen(fileName, "r");
	
	while(fgets(line,1024,file))
    {    
		lineCnt++;
		// remove comments
		if( (line[0]=='/') && (line[0]=='/') )
			continue;

		// remove inline comments
		for(q1=(int)strlen(line)-1;q1>=0;q1--)
		{
			if(line[q1]==';')
			{	line[q1]=0;
				break;
			}
			line[q1]=0;
		}
      
		strcpy_s(lineRaw, maxInputSz, line);

		if(q1==-1)
		{	if(strcmp(line,"")!=0) // if not comment or empty line
			{	sprintf_s(errmsg, "No semicolon on line %d(%s) ... skipping\n",lineCnt,lineRaw);
				util_warning(errmsg);
			}	
			continue;
		}			
		for(q1--;q1>=0;q1--)
			if(iswhite(line[q1]))
				line[q1]=0;
			else 
				break;
		if(q1==-1)
		{	sprintf_s(errmsg, "No cgdata on line %d(%s) ... skipping\n",lineCnt,lineRaw);
			util_warning(errmsg);
			continue;
		}
    
		numw=0;
    
		if(line[q1]=='"')
		{
			line[q1]=0;
			for(q1--;q1>=0;q1--)if(line[q1]=='"')break;
			if(q1==-1)
			{	sprintf_s(errmsg, "Expected string on line %d(%s), no matching \" ... skipping\n",lineCnt,lineRaw);
				util_warning(errmsg);
				continue;
			}
			line[q1]=0;
		}
		else if(line[q1]=='\'')
		{
			int cq1=q1;
			line[q1]=0;
			for(q1--;q1>=0;q1--)if(line[q1]=='\'')break;
			if(q1==-1)
			{	sprintf_s(errmsg, "Expected char on line %d(%s), no matching \' ... skipping\n",lineCnt,lineRaw);
				util_warning(errmsg);
				continue;
			}
			line[q1]=0;
			if(cq1-q1!=2)
			{	sprintf_s(errmsg, "Char needs to have length=1 %d(%s), no matching \' ... skipping\n",lineCnt,lineRaw);
				util_warning(errmsg);
				continue;
			}
		}
		else 
		{	for(;q1>=0;q1--)
				if(iswhite(line[q1]))
				{	line[q1]=0;
					break;
				}
		}
    
		wordPtr[numw++]=line+q1+1;
    
		for(q1--;q1>=0;q1--)
		{	if(q1==0||(iswhite(line[q1-1])&&(!iswhite(line[q1]))))
				wordPtr[numw++]=line+q1;
			if(iswhite(line[q1]))
				line[q1]=0;
		}
    
		//printf("------------------------------------\n");
		//for(q1=0;q1<numw;q1++)printf("%d: (%s)\n",q1,wordPtr[q1]);
		//for(q1=0;q1<icnt;q1++)printf("input:%d: (%s)\n",q1,input[q1]);

    
		if(numw!=4)
		{	sprintf_s(errmsg, "Not enough words (4 expected, %d got) on line %d (%s) ... skipping\n",numw,lineCnt,lineRaw);
			util_warning(errmsg);
			continue;
		}

		if(strcmp(wordPtr[1],"="))
		{	sprintf_s(errmsg, "Not equal sign as word 3 on line %d(%s) ... skipping\n",lineCnt,lineRaw);
			util_warning(errmsg);
			continue;
		}    
    
		/*if(strcmp(wordPtr[2],input[1]) || strcmp(wordPtr[3],input[0]))
		{
			sprintf_s(errmsg, "INFO: type/names ([%s] [%s]) and([%s] [%s]) don't match on line %d(%s) ... skipping\n",wordPtr[3],wordPtr[2],input[0],input[1],lineCnt,lineRaw);
			util_warning(errmsg);
			continue;
		} */   
    
		// Parse for variable values
		if(!strcmp(wordPtr[2], input[1])) // match variable name
		{
			// Check variable type			
			if(!strcmp(wordPtr[3],"int"))
			{	sscanf_s(wordPtr[0],"%d",(int*)var);
				err=0;
				break;
			}
			else if(!strcmp(wordPtr[3],"double"))
			{	sscanf_s(wordPtr[0],"%lf",(double*)var);
				err=0;
				break;
			}
			else if(!strcmp(wordPtr[3],"char"))
			{	*((char*)var)=wordPtr[0][0];
				err=0;
				break;
			}
			else if(!strcmp(wordPtr[3],"char*"))
			{	char*t=(char*)malloc((int)strlen(wordPtr[0])+1);
				strcpy_s(t, (int)strlen(wordPtr[0])+1, wordPtr[0]);
				*((char**)var)=t;
				err=0;
				break;
			}
			else if(!strcmp(wordPtr[3],"bool"))
			{
				if(!strcmp("true",wordPtr[0]))
				{	*((bool*)var)=true;
					err=0;
					break;
				}
				else if(!strcmp("false",wordPtr[0]))
				{	*((bool*)var)=false;
					err=0;
					break;
				}
				else 
				{	sprintf_s(errmsg, "Wrong bool literal on line %d(%s) ... skipping\n",lineCnt,lineRaw);
					util_warning(errmsg);
					continue;
				}
			}
			else
			{	sprintf_s(errmsg, "Type (%s) unrecognized on line %d(%s) ... skipping\n",wordPtr[3],lineCnt,lineRaw);
				util_warning(errmsg);
				continue;
			}
		}
    }
	fclose(file);
	if(err!=0)
	{	sprintf_s(errmsg, "Variable (%s %s) not found ... skipping\n", input[0], input[1]);
		util_warning(errmsg);
	}
  return err;
}

// Read options from the config file
cgOption* readOptions(const char* filename)
{
	// Use modes
	util_config(filename, "bool USEGLOVE", &option.USEGLOVE);
	util_config(filename, "bool STREAM_2_VIZ", &option.STREAM_2_VIZ);
	util_config(filename, "bool STREAM_2_DRIVER", &option.STREAM_2_DRIVER);
	util_config(filename, "bool HIRES_DATA", &option.HIRES_DATA);
	
	// Glove variables
	util_config(filename, "char* glove_port", &option.glove_port);
	util_config(filename, "int baudRate", &option.baudRate);
	util_config(filename, "int rawSenor_n", &option.rawSenor_n);
	util_config(filename, "bool updateRawRange", &option.updateRawRange);

	// Hand
	util_config(filename, "char* modelFile", &option.modelFile);
	util_config(filename, "char* logFile", &option.logFile);
	util_config(filename, "int calibSenor_n", &option.calibSenor_n);
	util_config(filename, "char* driver_ip", &option.driver_ip);
	util_config(filename, "char* driver_port", &option.driver_port);

	// Mujoco
	util_config(filename, "char* viz_ip", &option.viz_ip);
	util_config(filename, "int skip", &option.skip);

	// Inverse Kinematics
	util_config(filename, "char* ik_body_name", &option.ik_body_name);
	util_config(filename, "char* ik_pos_tolerance", &option.ik_pos_tolerance);
	util_config(filename, "char* ik_max_steps", &option.ik_max_steps);

	// Calibration 
	util_config(filename, "char* calibFile", &option.calibFile);
	util_config(filename, "char* userRangeFile", &option.userRangeFile);
	util_config(filename, "char* handRangeFile", &option.handRangeFile);

	return &option;
}

