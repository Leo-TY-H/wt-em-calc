// Stream immutable Git input trees into a deduplicated POSIX tar archive.
// Pipe to mksquashfs -tar: no extracted per-version datasets or Python file loop.
#include <git2.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <unordered_map>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdint>
static git_repository* repo;
static std::unordered_map<std::string,uint64_t> objects;
static uint64_t written=0,files=0,input_bytes=0,unique_bytes=0;
static const uint64_t ceiling=12000000000ULL;
static void check(int result){if(result<0)throw std::runtime_error(git_error_last()?git_error_last()->message:"Git error");}
static void output(const void* bytes,size_t size){
  if(written+size>ceiling)throw std::runtime_error("Archive stream exceeds finite 12 GB ceiling");
  if(fwrite(bytes,1,size,stdout)!=size)throw std::runtime_error("Archive write failed");written+=size;
}
static void octal(char* field,size_t size,uint64_t n){snprintf(field,size,"%0*llo",int(size-1),(unsigned long long)n);}
static void header(std::string name,uint64_t size,char type='0',const std::string& link=""){
  std::array<char,512> h{};
  if(name.size()>100){auto at=name.rfind('/',155);if(at==std::string::npos||name.size()-at-1>100)throw std::runtime_error("Path exceeds ustar limit");memcpy(h.data()+345,name.data(),at);name=name.substr(at+1);}
  memcpy(h.data(),name.data(),name.size());octal(h.data()+100,8,0644);octal(h.data()+108,8,0);octal(h.data()+116,8,0);
  octal(h.data()+124,12,size);octal(h.data()+136,12,0);memset(h.data()+148,' ',8);h[156]=type;
  if(link.size()>100)throw std::runtime_error("Link exceeds ustar limit");memcpy(h.data()+157,link.data(),link.size());
  memcpy(h.data()+257,"ustar",5);memcpy(h.data()+263,"00",2);
  unsigned sum=0;for(unsigned char c:h)sum+=c;snprintf(h.data()+148,7,"%06o",sum);h[155]=' ';
  output(h.data(),h.size());
}
static std::string prefix,error;
static int entry(const char* root,const git_tree_entry* e,void*){
  try{
    if(git_tree_entry_type(e)==GIT_OBJECT_TREE)return std::string(root)+git_tree_entry_name(e)=="native"?1:0;
    auto mode=git_tree_entry_filemode(e);
    if(git_tree_entry_type(e)!=GIT_OBJECT_BLOB||(mode!=GIT_FILEMODE_BLOB&&mode!=GIT_FILEMODE_BLOB_EXECUTABLE))throw std::runtime_error("Non-regular dataset entry");
    std::string name=std::string(root)+git_tree_entry_name(e);
    if(name.empty()||name[0]=='/'||name.find("\\")!=std::string::npos||name.find("../")!=std::string::npos)throw std::runtime_error("Unsafe dataset path");
    char hash[GIT_OID_HEXSZ+1];git_oid_tostr(hash,sizeof(hash),git_tree_entry_id(e));std::string object="objects/"+std::string(hash,2)+"/"+hash;
    auto existing=objects.find(hash);uint64_t size;
    if(existing==objects.end()){
      git_blob* blob=nullptr;check(git_blob_lookup(&blob,repo,git_tree_entry_id(e)));size=git_blob_rawsize(blob);objects.emplace(hash,size);
      header(object,size);output(git_blob_rawcontent(blob),size);std::array<char,512> pad{};output(pad.data(),(512-size%512)%512);unique_bytes+=size;git_blob_free(blob);
    }else size=existing->second;
    input_bytes+=size;files++;header(prefix+name,0,'1',object);return 0;
  }catch(const std::exception& ex){error=ex.what();return -1;}
}
int main(int argc,char** argv){
  try{
    if(argc<3)throw std::runtime_error("Usage: dataset_pack STORE VERSION [VERSION ...]");
    git_libgit2_init();check(git_repository_open_bare(&repo,argv[1]));
    for(int i=2;i<argc;i++){
      std::string spec=argv[i],version=spec.substr(0,spec.find('=')),ref=spec.find('=')==std::string::npos?version:spec.substr(spec.find('=')+1);
      if(version.find_first_not_of("0123456789.")!=std::string::npos)throw std::runtime_error("Invalid version");
      if(ref!=version&&(ref.size()!=40||ref.find_first_not_of("0123456789abcdef")!=std::string::npos))throw std::runtime_error("Invalid immutable tree");
      git_object* object=nullptr;check(git_revparse_single(&object,repo,(ref+"^{tree}").c_str()));prefix=version+"/references/";
      auto result=git_tree_walk(reinterpret_cast<git_tree*>(object),GIT_TREEWALK_PRE,entry,nullptr);git_object_free(object);
      if(result<0)throw std::runtime_error(error.empty()?"Tree walk failed":error);
      fprintf(stderr,"{\"version\":\"%s\",\"files\":%llu,\"stream_bytes\":%llu}\n",version.c_str(),(unsigned long long)files,(unsigned long long)written);
    }
    std::array<char,1024> end{};output(end.data(),end.size());if(fflush(stdout))throw std::runtime_error("Flush failed");
    fprintf(stderr,"{\"versions\":%d,\"files\":%llu,\"input_bytes\":%llu,\"unique_objects\":%zu,\"unique_bytes\":%llu,\"stream_bytes\":%llu}\n",argc-2,(unsigned long long)files,(unsigned long long)input_bytes,objects.size(),(unsigned long long)unique_bytes,(unsigned long long)written);
    git_repository_free(repo);git_libgit2_shutdown();return 0;
  }catch(const std::exception& ex){fprintf(stderr,"%s\n",ex.what());return 1;}
}
