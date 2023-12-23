#pragma once

#define SLOG_ERROR(...) std::fprintf(stderr, "ERROR: " __VA_ARGS__)
#define SLOG_WARN(...) std::fprintf(stderr, "WARN: " __VA_ARGS__)
#define SLOG_INFO(...) std::fprintf(stderr, "INFO: " __VA_ARGS__)
#define SLOG_DEBUG(...) std::fprintf(stderr, "DEBUG: " __VA_ARGS__)
