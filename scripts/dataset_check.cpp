// Verify every mounted version file against its immutable Git blob, hashing
// shared inodes once. Retains scalar evidence only; never copies input data.
#include <git2.h>
#include <sys/stat.h>
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <cstdio>
#include <cstdint>
struct Object {dev_t device;ino_t inode;uint64_t size;};
static std::unordered_map<std::string,Object> checked;
static std::string mount_root,prefix,failure;
static uint64_t files=0,bytes=0,unique_bytes=0;
static int entry(const char* root,const git_tree_entry* e,void*){
 try{
  if(git_tree_entry_type(e)==GIT_OBJECT_TREE)return std::string(root)+git_tree_entry_name(e)=="native"?1:0;
  if(git_tree_entry_type(e)!=GIT_OBJECT_BLOB)throw std::runtime_error("Non-regular input");
  char hash[GIT_OID_HEXSZ+1];git_oid_tostr(hash,sizeof(hash),git_tree_entry_id(e));auto it=checked.find(hash);
  if(it==checked.end()){
   auto object=mount_root+"/objects/"+std::string(hash,2)+"/"+hash;struct stat st{};if(lstat(object.c_str(),&st)||!S_ISREG(st.st_mode))throw std::runtime_error("Missing object "+object);
   git_oid found;if(git_odb_hashfile(&found,object.c_str(),GIT_OBJECT_BLOB)<0||!git_oid_equal(&found,git_tree_entry_id(e)))throw std::runtime_error("Object checksum mismatch "+object);
   it=checked.emplace(hash,Object{st.st_dev,st.st_ino,uint64_t(st.st_size)}).first;unique_bytes+=st.st_size;
  }
  auto path=prefix+root+git_tree_entry_name(e);struct stat st{};
  if(lstat(path.c_str(),&st)||!S_ISREG(st.st_mode)||st.st_dev!=it->second.device||st.st_ino!=it->second.inode||uint64_t(st.st_size)!=it->second.size)throw std::runtime_error("Dataset file mismatch "+path);
  files++;bytes+=st.st_size;return 0;
 }catch(const std::exception& e){failure=e.what();return -1;}
}
int main(int argc,char** argv){
 try{
  if(argc<4)throw std::runtime_error("Usage: dataset_check STORE MOUNT VERSION ...");git_libgit2_init();git_repository* repo=nullptr;
  if(git_repository_open_bare(&repo,argv[1])<0)throw std::runtime_error("Cannot open store");mount_root=argv[2];
  for(int i=3;i<argc;i++){git_object* tree=nullptr;std::string spec=argv[i],version=spec.substr(0,spec.find('=')),ref=spec.find('=')==std::string::npos?version:spec.substr(spec.find('=')+1);prefix=mount_root+"/"+version+"/references/";
   if(version.find_first_not_of("0123456789.")!=std::string::npos)throw std::runtime_error("Invalid version");
   if(ref!=version&&(ref.size()!=40||ref.find_first_not_of("0123456789abcdef")!=std::string::npos))throw std::runtime_error("Invalid immutable tree");
   if(git_revparse_single(&tree,repo,(ref+"^{tree}").c_str())<0)throw std::runtime_error("Unknown input tree");
   int result=git_tree_walk(reinterpret_cast<git_tree*>(tree),GIT_TREEWALK_PRE,entry,nullptr);git_object_free(tree);if(result<0)throw std::runtime_error(failure);
  }
  printf("{\"passed\":true,\"versions\":%d,\"files\":%llu,\"logical_bytes\":%llu,\"unique_blobs\":%zu,\"unique_bytes\":%llu}\n",argc-3,(unsigned long long)files,(unsigned long long)bytes,checked.size(),(unsigned long long)unique_bytes);
  git_repository_free(repo);git_libgit2_shutdown();return 0;
 }catch(const std::exception& e){fprintf(stderr,"%s\n",e.what());return 1;}
}
