package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class LogFolderTest {
    @TempDir
    Path folder;

    private File log(String name, int bytes, long modified) throws IOException {
        File file = folder.resolve(name).toFile();
        Files.write(file.toPath(), new byte[bytes]);
        assertTrue(file.setLastModified(modified));
        return file;
    }

    @Test
    void trimDeletesTheOldestLogsUntilUnderBudget() throws IOException {
        File oldest = log("a.wpilog", 400, 1_000_000L);
        File middle = log("b.wpilog", 400, 2_000_000L);
        File newest = log("c.wpilog", 400, 3_000_000L);
        File other = log("notes.txt", 5_000, 500_000L);
        LogFolder.trim(folder.toFile(), 900);
        assertFalse(oldest.exists());
        assertTrue(middle.exists());
        assertTrue(newest.exists());
        assertTrue(other.exists());
    }

    @Test
    void trimKeepsEverythingUnderBudget() throws IOException {
        log("a.wpilog", 100, 1_000_000L);
        log("b.wpilog", 100, 2_000_000L);
        LogFolder.trim(folder.toFile(), 1_000);
        assertEquals(2, folder.toFile().listFiles().length);
    }
}
