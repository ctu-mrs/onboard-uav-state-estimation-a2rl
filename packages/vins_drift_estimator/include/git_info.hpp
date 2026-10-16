#pragma once

#include <string>
#include <sstream>
#include <stdexcept>
#include <cstdio>
#include <array>
#include <tuple>
#include <filesystem>

#define BUFFER_SIZE 102400  


namespace git_info {

// Shell command execution with error and size handling
inline std::string safe_exec(const char* cmd) {
    std::array<char, 256> buffer;
    std::string result;

    FILE* pipe = popen(cmd, "r");
    if (!pipe) { 
      throw std::runtime_error("popen() failed");
    }

    try {
        while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
            result += buffer.data();
            if (result.size() > BUFFER_SIZE) {
                result += "\n[Output truncated]\n";
                break;
            }
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }

    int rc = pclose(pipe);
    if (rc != 0) {
        result += "\n[Non-zero return code]\n";
    }

    return result;
}

// Remove trailing newline from a string
inline std::string removeTrailingNewline(std::string str) {
    if (!str.empty() && str.back() == '\n') {
        str.pop_back();
    }
    return str;
}

// Get the source directory of this file
inline std::string getSourceDirectory(const std::string file_path) {
    namespace fs = std::filesystem;

    fs::path path(file_path);

    if (fs::is_directory(path)) {
        // Already a directory
        return path.string();
    } else if (fs::is_regular_file(path)) {
        // Extract parent directory from file path
        return path.parent_path().string();
    } else {
        // Not a valid path — fallback to current dir if no slash found 
        return "."; 
    }
}

// Check if this is a Git repository
inline bool isGitRepo(std::string repo_path) {
    try {
        std::string cmd = "cd " + repo_path + "; git rev-parse --is-inside-work-tree 2>/dev/null";
        std::string result = safe_exec(cmd.c_str());
        return result.find("true") != std::string::npos;
    } catch (...) {
        return false;
    }
}

// Collect commit hash and local changes
inline std::tuple<std::string, std::string> getGitMetadata(std::string file_path) {
    std::string commit_hash = "";
    std::string git_changes = "";
    std::string source_directory = getSourceDirectory(file_path);
    std::string cd_cmd = "cd " + source_directory;
    if (isGitRepo(source_directory)) {
        // Get commit hash   
        try {
            std::string cmd = cd_cmd + "; git rev-parse HEAD";
            commit_hash = safe_exec(cmd.c_str());
            commit_hash = removeTrailingNewline(commit_hash);
        } catch (const std::exception& e) {
            commit_hash = "Could not retrieve commit hash. Error: " + std::string(e.what());
        }
        // Get git local changes
        try {
            std::string cmd = cd_cmd + "; git status --porcelain";
            git_changes = safe_exec(cmd.c_str());            
            git_changes = removeTrailingNewline(git_changes);
            if (git_changes.empty()) {
                git_changes = "Clean";
            }
        } catch (const std::exception& e) {
            git_changes = "Could not retrieve git changes. Error: " + std::string(e.what());
        }
    } else {
      commit_hash = "Not a git repository";
      git_changes = "Not a git repository";
    }

    return std::make_tuple(commit_hash, git_changes);
}

} // namespace git_info

