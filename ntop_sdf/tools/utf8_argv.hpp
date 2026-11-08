#pragma once

#include <stdexcept>
#include <string>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>
#endif

// nTop Core expects UTF-8 paths. Windows main() arguments use the process code
// page, so retrieve the corresponding wide argument before conversion.
inline std::string ntop_path_argument(int index,const char* fallback) {
#ifdef _WIN32
  int count=0;
  wchar_t** args=CommandLineToArgvW(GetCommandLineW(),&count);
  if(!args || index<0 || index>=count) {
    if(args) LocalFree(args);
    throw std::runtime_error("cannot read Windows command line");
  }
  int bytes=WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,args[index],-1,nullptr,0,nullptr,nullptr);
  if(bytes<=0) { LocalFree(args); throw std::runtime_error("invalid Unicode command-line path"); }
  std::string result(static_cast<std::size_t>(bytes),'\0');
  int written=WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,args[index],-1,result.data(),bytes,nullptr,nullptr);
  LocalFree(args);
  if(!written) throw std::runtime_error("cannot convert command-line path to UTF-8");
  result.pop_back();
  return result;
#else
  (void)index;
  return fallback;
#endif
}
