/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Filesystem.h"

#include "AP_Filesystem_config.h"
#include <AP_HAL/HAL.h>
#include <AP_HAL/Util.h>
#include <AP_Math/AP_Math.h>

static AP_Filesystem fs;

// create exactly one "local" filesystem:
#if AP_FILESYSTEM_FATFS_ENABLED
#include "AP_Filesystem_FATFS.h"
static AP_Filesystem_FATFS fs_local;
#elif AP_FILESYSTEM_ESP32_ENABLED
#include "AP_Filesystem_ESP32.h"
static AP_Filesystem_ESP32 fs_local;
#elif AP_FILESYSTEM_LITTLEFS_ENABLED
#include "AP_Filesystem_FlashMemory_LittleFS.h"
static AP_Filesystem_FlashMemory_LittleFS fs_local;
#elif AP_FILESYSTEM_POSIX_ENABLED
#include "AP_Filesystem_posix.h"
static AP_Filesystem_Posix fs_local;
#else
static AP_Filesystem_Backend fs_local;
int errno;
#endif

#if AP_FILESYSTEM_ROMFS_ENABLED
#include "AP_Filesystem_ROMFS.h"
static AP_Filesystem_ROMFS fs_romfs;
#endif

#if AP_FILESYSTEM_PARAM_ENABLED
#include "AP_Filesystem_Param.h"
static AP_Filesystem_Param fs_param;
#endif

#if AP_FILESYSTEM_SYS_ENABLED
#include "AP_Filesystem_Sys.h"
static AP_Filesystem_Sys fs_sys;
#endif

#if AP_FILESYSTEM_MISSION_ENABLED
#include "AP_Filesystem_Mission.h"
static AP_Filesystem_Mission fs_mission;
#endif

/*
  mapping from filesystem prefix to backend
 */
const AP_Filesystem::Backend AP_Filesystem::backends[] = {
    { nullptr, fs_local },
#if AP_FILESYSTEM_ROMFS_ENABLED
    { "@ROMFS/", fs_romfs },
#endif
#if AP_FILESYSTEM_PARAM_ENABLED
    { "@PARAM/", fs_param },
#endif
#if AP_FILESYSTEM_SYS_ENABLED
    { "@SYS/", fs_sys },
    { "@SYS", fs_sys },
#endif
#if AP_FILESYSTEM_MISSION_ENABLED
    { "@MISSION/", fs_mission },
#endif
};

extern const AP_HAL::HAL& hal;

#define MAX_FD_PER_BACKEND 256U
#define NUM_BACKENDS ARRAY_SIZE(backends)
#define LOCAL_BACKEND backends[0]
#define BACKEND_IDX(backend) (&(backend) - &backends[0])

/*
  find backend by path
 */
const AP_Filesystem::Backend &AP_Filesystem::backend_by_path(const char *&path) const
{
    for (uint8_t i=1; i<NUM_BACKENDS; i++) {
        const uint8_t plen = strlen(backends[i].prefix);
        if (strncmp(path, backends[i].prefix, plen) == 0) {
            path += plen;
            return backends[i];
        }
    }
    // default to local filesystem
    return LOCAL_BACKEND;
}

/*
  return backend by file descriptor
 */
const AP_Filesystem::Backend &AP_Filesystem::backend_by_fd(int &fd) const
{
    if (fd < 0 || uint32_t(fd) >= NUM_BACKENDS*MAX_FD_PER_BACKEND) {
        return LOCAL_BACKEND;
    }
    const uint8_t idx = uint32_t(fd) / MAX_FD_PER_BACKEND;
    fd -= idx * MAX_FD_PER_BACKEND;
    return backends[idx];
}

int AP_Filesystem::open(const char *fname, int flags, bool allow_absolute_paths)
{
    const Backend &backend = backend_by_path(fname);
    int fd = backend.fs.open(fname, flags, allow_absolute_paths);
    if (fd < 0) {
        return -1;
    }
    if (uint32_t(fd) >= MAX_FD_PER_BACKEND) {
        backend.fs.close(fd);
        errno = ERANGE;
        return -1;
    }
    // offset fd so we can recognise the backend
    const uint8_t idx = (&backend - &backends[0]);
    fd += idx * MAX_FD_PER_BACKEND;
    return fd;
}

int AP_Filesystem::close(int fd)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.close(fd);
}

int32_t AP_Filesystem::read(int fd, void *buf, uint32_t count)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.read(fd, buf, count);
}

int32_t AP_Filesystem::write(int fd, const void *buf, uint32_t count)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.write(fd, buf, count);
}

int AP_Filesystem::fsync(int fd)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.fsync(fd);
}

int32_t AP_Filesystem::lseek(int fd, int32_t offset, int seek_from)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.lseek(fd, offset, seek_from);
}

int AP_Filesystem::stat(const char *pathname, struct stat *stbuf)
{
    const Backend &backend = backend_by_path(pathname);
    return backend.fs.stat(pathname, stbuf);
}

int AP_Filesystem::unlink(const char *pathname)
{
    const Backend &backend = backend_by_path(pathname);
    return backend.fs.unlink(pathname);
}

int AP_Filesystem::mkdir(const char *pathname)
{
    const Backend &backend = backend_by_path(pathname);
    return backend.fs.mkdir(pathname);
}

int AP_Filesystem::rename(const char *oldpath, const char *newpath)
{
    const Backend &backend = backend_by_path(oldpath);
    return backend.fs.rename(oldpath, newpath);
}

AP_Filesystem::DirHandle *AP_Filesystem::opendir(const char *pathname)
{
    const Backend &backend = backend_by_path(pathname);
    DirHandle *h = new DirHandle;
    if (!h) {
        return nullptr;
    }
    h->dir = backend.fs.opendir(pathname);
    if (h->dir == nullptr) {
        delete h;
        return nullptr;
    }
    h->fs_index = BACKEND_IDX(backend);
    return h;
}

struct dirent *AP_Filesystem::readdir(DirHandle *dirp)
{
    if (!dirp) {
        return nullptr;
    }
    const Backend &backend = backends[dirp->fs_index];
    return backend.fs.readdir(dirp->dir);
}

int AP_Filesystem::closedir(DirHandle *dirp)
{
    if (!dirp) {
        return -1;
    }
    const Backend &backend = backends[dirp->fs_index];
    int ret = backend.fs.closedir(dirp->dir);
    delete dirp;
    return ret;
}

// return free disk space in bytes
int64_t AP_Filesystem::disk_free(const char *path)
{
    const Backend &backend = backend_by_path(path);
    return backend.fs.disk_free(path);
}

// return total disk space in bytes
int64_t AP_Filesystem::disk_space(const char *path)
{
    const Backend &backend = backend_by_path(path);
    return backend.fs.disk_space(path);
}


/*
  set mtime on a file
 */
bool AP_Filesystem::set_mtime(const char *filename, const uint32_t mtime_sec)
{
    const Backend &backend = backend_by_path(filename);
    return backend.fs.set_mtime(filename, mtime_sec);
}

// if filesystem is not running then try a remount
bool AP_Filesystem::retry_mount(void)
{
    return LOCAL_BACKEND.fs.retry_mount();
}

// unmount filesystem for reboot
void AP_Filesystem::unmount(void)
{
    return LOCAL_BACKEND.fs.unmount();
}

/*
  load a file to memory as a single chunk. Use only for small files
 */
FileData *AP_Filesystem::load_file(const char *filename)
{
    const Backend &backend = backend_by_path(filename);
    return backend.fs.load_file(filename);
}

// returns null-terminated string; cr or lf terminates line
bool AP_Filesystem::fgets(char *buf, uint8_t buflen, int fd)
{
    const Backend &backend = backend_by_fd(fd);

    uint8_t i = 0;
    for (; i<buflen-1; i++) {
        if (backend.fs.read(fd, &buf[i], 1) <= 0) {
            if (i==0) {
                return false;
            }
            break;
        }
        if (buf[i] == '\r' || buf[i] == '\n') {
            break;
        }
    }
    buf[i] = '\0';
    return true;
}

// run crc32 over file with given name, returns true if successful
bool AP_Filesystem::crc32(const char *fname, uint32_t& checksum)
{
    // Ensure value is initialized
    checksum = 0;

    // Open file in readonly mode
    int fd = open(fname, O_RDONLY);
    if (fd == -1) {
        return false;
    }

    // Buffer to store data temporarily
    const ssize_t buff_len = 64;
    uint8_t buf[buff_len];

    // Read into buffer and run crc
    ssize_t read_size;
    do {
        read_size = read(fd, buf, buff_len);
        if (read_size == -1) {
            // Read error, note that we have changed the checksum value in this case
            close(fd);
            return false;
        }
        checksum = crc_crc32(checksum, buf, MIN(read_size, buff_len));
    } while (read_size > 0);

    close(fd);

    return true;
}


#if AP_FILESYSTEM_FORMAT_ENABLED
// format filesystem
bool AP_Filesystem::format(void)
{
    if (hal.util->get_soft_armed()) {
        return false;
    }
    return LOCAL_BACKEND.fs.format();
}
AP_Filesystem_Backend::FormatStatus AP_Filesystem::get_format_status(void) const
{
    return LOCAL_BACKEND.fs.get_format_status();
}
#endif

/*
  stat wrapper for scripting
 */
bool AP_Filesystem::stat(const char *pathname, stat_t &stbuf)
{
    struct stat st;
    if (fs.stat(pathname, &st) != 0) {
        return false;
    }
    stbuf.size = st.st_size;
    stbuf.mode = st.st_mode;
    // these wrap in 2038
    stbuf.atime = st.st_atime;
    stbuf.ctime = st.st_ctime;
    stbuf.mtime = st.st_mtime;
    return true;
}

// get_singleton for scripting
AP_Filesystem *AP_Filesystem::get_singleton(void)
{
    return &fs;
}

namespace AP
{
AP_Filesystem &FS()
{
    return fs;
}
}

