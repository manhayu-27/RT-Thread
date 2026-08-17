package com.example.esp32monitor

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.app.AlertDialog
import android.content.Intent
import android.content.ActivityNotFoundException
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.VibrationEffect
import android.os.Vibrator
import android.provider.Settings
import android.speech.RecognitionListener
import android.speech.RecognizerIntent
import android.speech.SpeechRecognizer
import android.speech.tts.TextToSpeech
import android.speech.tts.UtteranceProgressListener
import android.speech.tts.Voice
import android.telephony.PhoneNumberUtils
import android.telephony.SmsManager
import android.text.InputType
import android.webkit.JavascriptInterface
import android.webkit.WebChromeClient
import android.webkit.WebView
import android.webkit.WebViewClient
import android.widget.Toast
import android.widget.EditText
import androidx.activity.ComponentActivity
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.io.FileWriter
import java.time.ZonedDateTime
import java.time.format.DateTimeFormatter
import java.util.Locale
import java.util.UUID
import java.net.HttpURLConnection
import java.net.URL
import android.location.Geocoder
import android.media.MediaPlayer
import android.media.AudioManager
import android.media.ToneGenerator
import android.util.Base64
import android.widget.LinearLayout

class MainActivity : ComponentActivity(), LocationListener {
    private companion object {
        const val ARK_CHAT_MODEL = "doubao-seed-2-0-mini-260215"
        // The previously configured 2.1-pro dated endpoint is not available to this Ark key.
        // Reuse the enabled chat model so report generation and chat have the same availability.
        const val ARK_REPORT_MODEL = ARK_CHAT_MODEL
        const val VOLC_TTS_URL = "https://openspeech.bytedance.com/api/v1/tts"
        const val DEFAULT_VOLC_TTS_VOICE = "zh_female_sajiaonvyou_moon_bigtts"
        const val AI_SYSTEM_PROMPT = "你是智能假肢设备的信号观察助手。只依据提供的数值、波形统计和姿态数据，给出2至4句简短、友好的工程观察。可以描述波形起伏、肌电相对活跃度、姿态变化和已触发的跌倒标志；数据不足时明确说明。严禁诊断、疾病名称、病因推测、健康结论、治疗、用药、康复处方、风险分级或恐吓性措辞；不要把一般信号波动说成异常。"
    }
    private lateinit var webView: WebView
    private val taskDir by lazy { File(filesDir, "data").apply { mkdirs() } }
    private var exportFile: File? = null
    private val preferences by lazy { getSharedPreferences("bioscope", MODE_PRIVATE) }
    @Volatile private var latestPhoneLocation: Location? = null
    private var pendingEmergencyPhone: String? = null
    private var speechRecognizer: SpeechRecognizer? = null
    private var textToSpeech: TextToSpeech? = null
    private var ttsAudioPlayer: MediaPlayer? = null
    private var fallToneGenerator: ToneGenerator? = null
    private var voiceStartPending = false
    private var voiceCancelRequested = false

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        applyBundledVolcTtsDefaults()
        webView = WebView(this).apply {
            settings.javaScriptEnabled = true
            settings.domStorageEnabled = true
            settings.allowFileAccess = true
            addJavascriptInterface(Bridge(), "AndroidHost")
            webViewClient = WebViewClient()
            webChromeClient = WebChromeClient()
            loadUrl("file:///android_asset/index.html")
        }
        setContentView(webView)
        textToSpeech = TextToSpeech(this) { status ->
            if (status == TextToSpeech.SUCCESS) {
                configureNaturalChineseVoice()
                textToSpeech?.setOnUtteranceProgressListener(object : UtteranceProgressListener() {
                    override fun onStart(utteranceId: String) = postJavascript("window.onAiSpeechState('speaking')")
                    override fun onDone(utteranceId: String) = postJavascript("window.onAiSpeechState('done')")
                    override fun onError(utteranceId: String) = postJavascript("window.onAiSpeechState('error')")
                })
            }
        }
    }

    override fun onDestroy() {
        (getSystemService(LOCATION_SERVICE) as LocationManager).removeUpdates(this)
        speechRecognizer?.destroy()
        ttsAudioPlayer?.release()
        fallToneGenerator?.release()
        textToSpeech?.shutdown()
        webView.destroy()
        super.onDestroy()
    }

    private fun now(): String = DateTimeFormatter.ISO_OFFSET_DATE_TIME.format(ZonedDateTime.now())

    private fun applyBundledVolcTtsDefaults() {
        if (BuildConfig.DEFAULT_VOLC_TTS_APP_ID.isBlank() || BuildConfig.DEFAULT_VOLC_TTS_ACCESS_TOKEN.isBlank()) return
        val editor = preferences.edit()
        val configurationVersion = BuildConfig.DEFAULT_VOLC_TTS_CONFIG_VERSION
        val needsMigration = configurationVersion.isNotBlank() &&
            preferences.getString("volcTtsDefaultVersion", "") != configurationVersion
        if (needsMigration || preferences.getString("volcTtsAppId", "").isNullOrBlank()) {
            editor.putString("volcTtsAppId", BuildConfig.DEFAULT_VOLC_TTS_APP_ID)
        }
        if (needsMigration || preferences.getString("volcTtsToken", "").isNullOrBlank()) {
            editor.putString("volcTtsToken", BuildConfig.DEFAULT_VOLC_TTS_ACCESS_TOKEN)
        }
        if (needsMigration || preferences.getString("volcTtsVoice", "").isNullOrBlank()) {
            editor.putString("volcTtsVoice", BuildConfig.DEFAULT_VOLC_TTS_VOICE.ifBlank { DEFAULT_VOLC_TTS_VOICE })
        }
        if (configurationVersion.isNotBlank()) editor.putString("volcTtsDefaultVersion", configurationVersion)
        editor.apply()
    }

    private fun configureNaturalChineseVoice() {
        val tts = textToSpeech ?: return
        tts.language = Locale.SIMPLIFIED_CHINESE
        tts.setSpeechRate(1.08f)
        tts.setPitch(1.0f)
        val voice = tts.voices
            ?.filter { it.locale.language == Locale.SIMPLIFIED_CHINESE.language && !it.isNetworkConnectionRequired }
            ?.sortedWith(compareByDescending<Voice> { it.quality }.thenBy { it.latency })
            ?.firstOrNull()
        if (voice != null) tts.voice = voice
    }

    private fun speakWithSystemVoice(text: String) {
        textToSpeech?.speak(text.take(800), TextToSpeech.QUEUE_FLUSH, null, "bioscope-ai")
    }

    private fun speakWithVolcTts(text: String): Boolean {
        val appId = preferences.getString("volcTtsAppId", "")?.trim().orEmpty()
        val token = preferences.getString("volcTtsToken", "")?.trim().orEmpty()
        if (appId.isBlank() || token.isBlank()) return false
        val voice = preferences.getString("volcTtsVoice", DEFAULT_VOLC_TTS_VOICE)?.trim()
            .takeUnless { it.isNullOrBlank() } ?: DEFAULT_VOLC_TTS_VOICE
        Thread {
            try {
                val request = JSONObject().apply {
                    put("app", JSONObject().put("appid", appId).put("token", token).put("cluster", "volcano_tts"))
                    put("user", JSONObject().put("uid", "esp32monitor"))
                    put("audio", JSONObject().put("voice_type", voice).put("encoding", "mp3")
                        .put("speed_ratio", 1.15).put("volume_ratio", 1.0).put("pitch_ratio", 1.03))
                    put("request", JSONObject().put("reqid", UUID.randomUUID().toString())
                        .put("text", text.take(800)).put("text_type", "plain").put("operation", "query")
                        .put("with_frontend", 1))
                }
                val connection = URL(VOLC_TTS_URL).openConnection() as HttpURLConnection
                val bytes = try {
                    connection.requestMethod = "POST"
                    connection.connectTimeout = 15000
                    connection.readTimeout = 30000
                    connection.setRequestProperty("Authorization", "Bearer;$token")
                    connection.setRequestProperty("Content-Type", "application/json")
                    connection.doOutput = true
                    connection.outputStream.bufferedWriter().use { it.write(request.toString()) }
                    val body = (if (connection.responseCode in 200..299) connection.inputStream else connection.errorStream)
                        .bufferedReader().use { it.readText() }
                    val response = JSONObject(body)
                    if (connection.responseCode !in 200..299 || response.optString("data").isBlank()) {
                        throw IllegalStateException(response.optString("message", "豆包语音合成失败"))
                    }
                    Base64.decode(response.getString("data"), Base64.DEFAULT)
                } finally {
                    connection.disconnect()
                }
                val audioFile = File(cacheDir, "volc_tts_${System.currentTimeMillis()}.mp3").apply { writeBytes(bytes) }
                runOnUiThread { playVolcTtsAudio(audioFile) }
            } catch (error: Exception) {
                postJavascript("window.onAiSpeechState('fallback', ${JSONObject.quote(error.message ?: "语音服务请求失败")})")
                runOnUiThread { speakWithSystemVoice(text) }
            }
        }.start()
        return true
    }

    private fun playVolcTtsAudio(audioFile: File) {
        ttsAudioPlayer?.release()
        ttsAudioPlayer = MediaPlayer().apply {
            setDataSource(audioFile.absolutePath)
            setOnPreparedListener { player ->
                player.start()
                postJavascript("window.onAiSpeechState('speaking')")
            }
            setOnCompletionListener { player ->
                player.release()
                if (ttsAudioPlayer === player) ttsAudioPlayer = null
                audioFile.delete()
                postJavascript("window.onAiSpeechState('done')")
            }
            setOnErrorListener { player, _, _ ->
                player.release()
                if (ttsAudioPlayer === player) ttsAudioPlayer = null
                audioFile.delete()
                postJavascript("window.onAiSpeechState('fallback')")
                false
            }
            prepareAsync()
        }
    }

    private fun postJavascript(script: String) {
        webView.post { if (!isFinishing && !isDestroyed) webView.evaluateJavascript(script, null) }
    }

    private fun taskFile(id: String, extension: String): File {
        require(id.matches(Regex("[A-Za-z0-9_-]+"))) { "invalid task id" }
        return File(taskDir, "$id.$extension")
    }

    private fun publicMeta(meta: JSONObject): JSONObject = JSONObject().apply {
        listOf("id", "name", "note", "status", "createdAt", "startedAt", "endedAt", "sampleRate", "sampleCount", "alarmCount", "durationSeconds")
            .forEach { key -> put(key, meta.opt(key)) }
    }

    private fun listTasks(): JSONObject {
        val tasks = taskDir.listFiles { file -> file.extension == "json" }?.mapNotNull {
            runCatching { publicMeta(JSONObject(it.readText())) }.getOrNull()
        }?.sortedByDescending { it.optString("createdAt") } ?: emptyList()
        return JSONObject().put("tasks", JSONArray().apply { tasks.forEach(::put) })
    }

    private fun createTask(body: JSONObject): JSONObject {
        val name = body.optString("name").trim()
        val note = body.optString("note").trim()
        require(name.isNotEmpty() && name.length <= 80 && note.length <= 500) { "任务名称或备注无效" }
        val id = "${System.currentTimeMillis()}_${UUID.randomUUID().toString().take(6)}"
        val meta = JSONObject().apply {
            put("id", id); put("name", name); put("note", note); put("status", "recording")
            put("createdAt", now()); put("startedAt", now()); put("startedAtMs", System.currentTimeMillis())
            put("endedAt", JSONObject.NULL); put("sampleRate", body.optInt("sampleRate", 500))
            put("sampleCount", 0); put("alarmCount", 0); put("durationSeconds", 0)
        }
        taskFile(id, "csv").writeText("sample_index,timestamp_us,ecg_mv,emg1_mv,emg2_mv,emg3_mv,gyro_x_dps,gyro_y_dps,gyro_z_dps,fall,roll_deg,pitch_deg,yaw_deg,temperature_c,alarm_flags\n")
        taskFile(id, "json").writeText(meta.toString())
        return publicMeta(meta)
    }

    private fun appendSamples(id: String, body: JSONObject): JSONObject {
        val rows = body.getJSONArray("samples")
        require(rows.length() <= 5000) { "samples too large" }
        val meta = JSONObject(taskFile(id, "json").readText())
        require(meta.optString("status") == "recording") { "任务未在采集中" }
        var count = meta.optInt("sampleCount")
        var alarms = meta.optInt("alarmCount")
        FileWriter(taskFile(id, "csv"), true).use { writer ->
            for (rowIndex in 0 until rows.length()) {
                val row = rows.getJSONArray(rowIndex)
                require(row.length() >= 14) { "sample format invalid" }
                val flags = row.optInt(13)
                writer.append(count.toString())
                for (column in 0 until 13) writer.append(',').append(row.opt(column).toString())
                writer.append(',').append(flags.toString()).append('\n')
                count++
                if (flags != 0) alarms++
            }
        }
        meta.put("sampleCount", count).put("alarmCount", alarms)
        taskFile(id, "json").writeText(meta.toString())
        return publicMeta(meta)
    }

    private fun stopTask(id: String): JSONObject {
        val meta = JSONObject(taskFile(id, "json").readText())
        meta.put("status", "completed").put("endedAt", now())
            .put("durationSeconds", ((System.currentTimeMillis() - meta.optLong("startedAtMs")) / 1000).coerceAtLeast(0))
        taskFile(id, "json").writeText(meta.toString())
        return publicMeta(meta)
    }

    private fun taskData(id: String): JSONObject {
        val meta = JSONObject(taskFile(id, "json").readText())
        val total = meta.optInt("sampleCount")
        val stride = maxOf(1, (total + 11999) / 12000)
        val samples = JSONArray()
        taskFile(id, "csv").useLines { lines ->
            lines.drop(1).forEachIndexed { index, line ->
                if (index % stride != 0 && index != total - 1) return@forEachIndexed
                val values = line.split(',')
                if (values.size != 15) return@forEachIndexed
                val row = JSONArray()
                row.put(values[0].toInt()).put(values[1].toLong())
                for (column in 2..13) row.put(values[column].toDouble())
                row.put(values[14].toInt())
                samples.put(row)
            }
        }
        return JSONObject().put("task", publicMeta(meta)).put("stride", stride).put("samples", samples)
    }

    private fun deleteTask(id: String): JSONObject {
        val meta = JSONObject(taskFile(id, "json").readText())
        require(meta.optString("status") != "recording") { "正在采集的任务不能删除" }
        taskFile(id, "json").delete(); taskFile(id, "csv").delete()
        return JSONObject().put("ok", true)
    }

    private fun taskSummary(id: String): String {
        val meta = JSONObject(taskFile(id, "json").readText())
        val sums = DoubleArray(4)
        val squares = DoubleArray(4)
        var count = 0
        var alarms = 0
        taskFile(id, "csv").useLines { lines -> lines.drop(1).forEach { line ->
            val values = line.split(',')
            if (values.size != 15) return@forEach
            for (index in 0..3) {
                val value = values[index + 2].toDouble()
                sums[index] += value; squares[index] += value * value
            }
            if (values[14].toInt() != 0) alarms++
            count++
        } }
        require(count > 0) { "该任务还没有可分析的信号数据" }
        fun stats(index: Int) = String.format(Locale.US, "mean=%.4f mV, rms=%.4f mV", sums[index] / count, kotlin.math.sqrt(squares[index] / count))
        return "任务：${meta.optString("name")}；样本数：$count；报警样本：$alarms；ECG ${stats(0)}；EMG1 ${stats(1)}；EMG2 ${stats(2)}；EMG3 ${stats(3)}。"
    }

    private fun callArk(messages: JSONArray, model: String): String {
        val key = preferences.getString("arkKey", "")?.trim().orEmpty()
        require(key.isNotEmpty()) { "请先点击 AI 状态配置 ARK_API_KEY" }
        val request = JSONObject().put("model", model).put("messages", messages).put("max_tokens", 1600)
        val connection = URL("https://ark.cn-beijing.volces.com/api/v3/chat/completions").openConnection() as HttpURLConnection
        return try {
            connection.requestMethod = "POST"; connection.connectTimeout = 15000; connection.readTimeout = 60000
            connection.setRequestProperty("Authorization", "Bearer $key")
            connection.setRequestProperty("Content-Type", "application/json")
            connection.doOutput = true
            connection.outputStream.bufferedWriter().use { it.write(request.toString()) }
            val body = (if (connection.responseCode in 200..299) connection.inputStream else connection.errorStream).bufferedReader().use { it.readText() }
            val response = JSONObject(body)
            if (connection.responseCode !in 200..299) throw IllegalStateException(response.optJSONObject("error")?.optString("message") ?: "AI 请求失败")
            response.getJSONArray("choices").getJSONObject(0).getJSONObject("message").getString("content")
        } finally { connection.disconnect() }
    }

    private inner class Bridge {
        @JavascriptInterface fun api(path: String, method: String, body: String): String = try {
            val result = when {
                path == "/api/health" -> JSONObject().put("ok", true)
                path == "/api/ai/status" -> JSONObject().put("configured", preferences.contains("arkKey"))
                    .put("chatModel", ARK_CHAT_MODEL).put("reportModel", ARK_REPORT_MODEL)
                    .put("volcTtsConfigured", preferences.contains("volcTtsAppId") && preferences.contains("volcTtsToken"))
                path == "/api/tasks" && method == "GET" -> listTasks()
                path == "/api/tasks" && method == "POST" -> createTask(JSONObject(body))
                Regex("/api/tasks/[^/]+/samples").matches(path) && method == "POST" -> appendSamples(path.split('/')[3], JSONObject(body))
                Regex("/api/tasks/[^/]+/stop").matches(path) && method == "POST" -> stopTask(path.split('/')[3])
                Regex("/api/tasks/[^/]+/data").matches(path) && method == "GET" -> taskData(path.split('/')[3])
                Regex("/api/tasks/[^/]+").matches(path) && method == "DELETE" -> deleteTask(path.split('/')[3])
                path == "/api/ai/report" && method == "POST" -> JSONObject().put("report", callArk(JSONArray()
                    .put(JSONObject().put("role", "system").put("content", AI_SYSTEM_PROMPT))
                    .put(JSONObject().put("role", "user").put("content", "根据以下采集统计给出简短工程观察，不要诊断或给医疗建议：" + taskSummary(JSONObject(body).getString("taskId")))), ARK_REPORT_MODEL))
                path == "/api/ai/realtime" && method == "POST" -> {
                    val context = JSONObject(body).getJSONObject("context")
                    val messages = JSONArray().put(JSONObject().put("role", "system").put("content", AI_SYSTEM_PROMPT))
                        .put(JSONObject().put("role", "user").put("content", "把以下实时数据压缩成一句友好观察，不做诊断：$context"))
                    JSONObject().put("summary", callArk(messages, ARK_CHAT_MODEL))
                }
                path == "/api/ai/chat" && method == "POST" -> {
                    val payload = JSONObject(body)
                    val messages = JSONArray().put(JSONObject().put("role", "system").put("content", AI_SYSTEM_PROMPT))
                    payload.optJSONObject("realtimeContext")?.let { messages.put(JSONObject().put("role", "system").put("content", "当前实时数据：$it")) }
                    payload.optString("taskId").takeIf { it.isNotBlank() }?.let { messages.put(JSONObject().put("role", "system").put("content", "任务统计：" + taskSummary(it))) }
                    messages.put(JSONObject().put("role", "user").put("content", payload.getString("message")))
                    JSONObject().put("answer", callArk(messages, ARK_CHAT_MODEL))
                }
                else -> JSONObject().put("error", "not found")
            }
            result.toString()
        } catch (error: Exception) {
            JSONObject().put("error", error.message ?: "操作失败").toString()
        }

        @JavascriptInterface fun startLocation() = runOnUiThread { requestPhoneLocation() }

        @JavascriptInterface fun startVoiceQuestion() = runOnUiThread { beginVoiceQuestion() }

        @JavascriptInterface fun stopVoiceQuestion() = runOnUiThread { speechRecognizer?.stopListening() }

        @JavascriptInterface fun cancelVoiceQuestion() = runOnUiThread { this@MainActivity.cancelVoiceQuestion() }

        @JavascriptInterface fun speak(text: String) = runOnUiThread {
            if (!speakWithVolcTts(text)) speakWithSystemVoice(text)
        }

        @JavascriptInterface fun stopSpeaking() = runOnUiThread {
            ttsAudioPlayer?.stop()
            ttsAudioPlayer?.release()
            ttsAudioPlayer = null
            textToSpeech?.stop()
        }

        @JavascriptInterface fun playFallAlarm() = runOnUiThread {
            val tone = fallToneGenerator ?: ToneGenerator(AudioManager.STREAM_ALARM, 100).also {
                fallToneGenerator = it
            }
            tone.startTone(ToneGenerator.TONE_SUP_ERROR, 260)
            webView.postDelayed({ tone.startTone(ToneGenerator.TONE_SUP_ERROR, 260) }, 330L)
            webView.postDelayed({ tone.startTone(ToneGenerator.TONE_SUP_ERROR, 300) }, 660L)
        }

        @JavascriptInterface fun requestAiChat(payloadJson: String) {
            Thread {
                val payload = runCatching { JSONObject(payloadJson) }.getOrElse {
                    postJavascript("window.onAiChatError(0, ${JSONObject.quote("AI 请求参数无效")})")
                    return@Thread
                }
                val requestId = payload.optLong("requestId")
                try {
                    val messages = JSONArray().put(JSONObject().put("role", "system").put("content", AI_SYSTEM_PROMPT))
                    payload.optJSONObject("realtimeContext")?.let { messages.put(JSONObject().put("role", "system").put("content", "当前实时数据：$it")) }
                    payload.optString("taskId").takeIf { it.isNotBlank() }?.let { messages.put(JSONObject().put("role", "system").put("content", "任务统计：" + taskSummary(it))) }
                    messages.put(JSONObject().put("role", "user").put("content", payload.getString("message")))
                    postJavascript("window.onAiChatResult($requestId, ${JSONObject.quote(callArk(messages, ARK_CHAT_MODEL))})")
                } catch (error: Exception) {
                    postJavascript("window.onAiChatError($requestId, ${JSONObject.quote(error.message ?: "AI 对话失败")})")
                }
            }.start()
        }

        @JavascriptInterface fun requestAiReport(payloadJson: String) {
            Thread {
                val payload = runCatching { JSONObject(payloadJson) }.getOrElse {
                    postJavascript("window.onAiReportError(0, ${JSONObject.quote("报告请求参数无效")})")
                    return@Thread
                }
                val requestId = payload.optLong("requestId")
                try {
                    val taskId = payload.getString("taskId")
                    val messages = JSONArray().put(JSONObject().put("role", "system").put("content", AI_SYSTEM_PROMPT))
                        .put(JSONObject().put("role", "user").put("content", "根据以下采集统计给出简短工程观察，不要诊断或给医疗建议：" + taskSummary(taskId)))
                    postJavascript("window.onAiReportResult($requestId, ${JSONObject.quote(callArk(messages, ARK_REPORT_MODEL))})")
                } catch (error: Exception) {
                    postJavascript("window.onAiReportError($requestId, ${JSONObject.quote(error.message ?: "报告生成失败")})")
                }
            }.start()
        }

        @JavascriptInterface fun requestRealtimeAi(contextJson: String) {
            Thread {
                try {
                    val context = JSONObject(contextJson)
                    val messages = JSONArray().put(JSONObject().put("role", "system").put("content", AI_SYSTEM_PROMPT))
                        .put(JSONObject().put("role", "user").put("content", "把以下实时数据压缩成一句友好观察，不做诊断：$context"))
                    val summary = callArk(messages, ARK_CHAT_MODEL)
                    webView.post { webView.evaluateJavascript("window.onRealtimeAiSummary(${JSONObject.quote(summary)})", null) }
                } catch (error: Exception) {
                    val message = error.message ?: "AI 实时解读失败"
                    webView.post {
                        webView.evaluateJavascript("window.onRealtimeAiError(${JSONObject.quote(message)})", null)
                    }
                }
            }.start()
        }

        @JavascriptInterface fun sendEmergencySms(phone: String) = runOnUiThread {
            requestEmergencySms(phone)
        }

        @JavascriptInterface fun configureAi() = runOnUiThread {
            val container = LinearLayout(this@MainActivity).apply {
                orientation = LinearLayout.VERTICAL
                val padding = (20 * resources.displayMetrics.density).toInt()
                setPadding(padding, 0, padding, 0)
            }
            fun input(hint: String, value: String, secret: Boolean = false) = EditText(this@MainActivity).apply {
                this.hint = hint
                setText(value)
                inputType = InputType.TYPE_CLASS_TEXT or if (secret) InputType.TYPE_TEXT_VARIATION_PASSWORD else InputType.TYPE_TEXT_VARIATION_NORMAL
                container.addView(this)
            }
            val arkInput = input("ARK_API_KEY（文本对话）", preferences.getString("arkKey", "") ?: "", true)
            val ttsAppIdInput = input("豆包语音 AppID（留空则用系统朗读）", preferences.getString("volcTtsAppId", "") ?: "")
            val ttsTokenInput = input("豆包语音 Access Token", preferences.getString("volcTtsToken", "") ?: "", true)
            val ttsVoiceInput = input("豆包音色 ID", preferences.getString("volcTtsVoice", DEFAULT_VOLC_TTS_VOICE) ?: DEFAULT_VOLC_TTS_VOICE)
            AlertDialog.Builder(this@MainActivity).setTitle("配置豆包 AI 与语音").setView(container)
                .setPositiveButton("仅保存到本机") { _, _ ->
                    preferences.edit()
                        .putString("arkKey", arkInput.text.toString().trim())
                        .putString("volcTtsAppId", ttsAppIdInput.text.toString().trim())
                        .putString("volcTtsToken", ttsTokenInput.text.toString().trim())
                        .putString("volcTtsVoice", ttsVoiceInput.text.toString().trim())
                        .apply()
                    webView.evaluateJavascript("checkAiStatus()", null)
                }
                .setNegativeButton("取消", null).show()
        }

        @JavascriptInterface fun exportCsv(id: String) = runOnUiThread {
            exportFile = taskFile(id, "csv")
            startActivityForResult(Intent(Intent.ACTION_CREATE_DOCUMENT).apply {
                type = "text/csv"; putExtra(Intent.EXTRA_TITLE, "$id.csv")
                addCategory(Intent.CATEGORY_OPENABLE)
            }, 7)
        }
    }

    private fun beginVoiceQuestion() {
        if (checkSelfPermission(Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
            voiceStartPending = true
            requestPermissions(arrayOf(Manifest.permission.RECORD_AUDIO), 9)
            return
        }
        voiceCancelRequested = false
        speechRecognizer?.destroy()
        val recognizer = try {
            when {
                Build.VERSION.SDK_INT >= Build.VERSION_CODES.S && SpeechRecognizer.isOnDeviceRecognitionAvailable(this) ->
                    SpeechRecognizer.createOnDeviceSpeechRecognizer(this)
                SpeechRecognizer.isRecognitionAvailable(this) -> SpeechRecognizer.createSpeechRecognizer(this)
                else -> {
                    launchSystemVoiceIntent()
                    return
                }
            }
        } catch (_: Exception) {
            launchSystemVoiceIntent()
            return
        }
        speechRecognizer = recognizer.apply {
            setRecognitionListener(object : RecognitionListener {
                override fun onReadyForSpeech(params: Bundle?) = Unit
                override fun onBeginningOfSpeech() = Unit
                override fun onRmsChanged(rmsdB: Float) = Unit
                override fun onBufferReceived(buffer: ByteArray?) = Unit
                override fun onEndOfSpeech() = Unit
                override fun onError(error: Int) {
                    if (!voiceCancelRequested) sendVoiceError(voiceErrorMessage(error))
                }
                override fun onResults(results: Bundle?) {
                    val text = results?.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION)?.firstOrNull()?.trim()
                    if (voiceCancelRequested) return
                    if (text.isNullOrBlank()) sendVoiceError("未识别到有效语音，请重试") else sendVoiceQuestion(text)
                }
                override fun onPartialResults(partialResults: Bundle?) = Unit
                override fun onEvent(eventType: Int, params: Bundle?) = Unit
            })
            startListening(Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH).apply {
                putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
                putExtra(RecognizerIntent.EXTRA_LANGUAGE, "zh-CN")
                putExtra(RecognizerIntent.EXTRA_PROMPT, "请说出你的问题")
            })
        }
    }

    private fun voiceErrorMessage(error: Int): String = when (error) {
        SpeechRecognizer.ERROR_AUDIO -> "麦克风采集失败，请检查系统麦克风权限"
        SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS -> "未授予麦克风权限，无法语音提问"
        SpeechRecognizer.ERROR_NETWORK, SpeechRecognizer.ERROR_NETWORK_TIMEOUT -> "语音服务网络不可用，请检查网络或启用离线语音输入"
        SpeechRecognizer.ERROR_NO_MATCH, SpeechRecognizer.ERROR_SPEECH_TIMEOUT -> "未识别到有效语音，请按住后清晰说出问题"
        SpeechRecognizer.ERROR_RECOGNIZER_BUSY -> "语音识别服务正忙，请稍后重试"
        SpeechRecognizer.ERROR_SERVER -> "系统语音识别服务暂不可用"
        else -> "语音识别失败（错误码 $error），请检查系统语音输入服务"
    }

    private fun sendVoiceQuestion(text: String) {
        postJavascript("window.onVoiceQuestion(${JSONObject.quote(text)})")
    }

    private fun cancelVoiceQuestion() {
        voiceCancelRequested = true
        speechRecognizer?.cancel()
        postJavascript("window.onVoiceCancelled()")
    }

    private fun launchSystemVoiceIntent() {
        try {
            startActivityForResult(Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH).apply {
                putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
                putExtra(RecognizerIntent.EXTRA_LANGUAGE, "zh-CN")
                putExtra(RecognizerIntent.EXTRA_PROMPT, "请说出你的问题")
            }, 10)
        } catch (_: ActivityNotFoundException) {
            sendVoiceError("未找到可供 App 调用的语音识别服务，请在系统设置启用语音输入服务")
        }
    }

    private fun sendVoiceError(message: String) {
        postJavascript("window.onVoiceError(${JSONObject.quote(message)})")
    }

    private fun requestPhoneLocation() {
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION), 6)
            return
        }
        val manager = getSystemService(LOCATION_SERVICE) as LocationManager
        if (!manager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
            Toast.makeText(this, "请打开手机定位服务", Toast.LENGTH_LONG).show()
            startActivity(Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS))
            return
        }
        manager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000L, 0f, this)
        manager.getLastKnownLocation(LocationManager.GPS_PROVIDER)?.let { location -> onLocationChanged(location) }
    }

    override fun onLocationChanged(location: Location) {
        latestPhoneLocation = Location(location)
        webView.post {
            webView.evaluateJavascript(
                String.format(Locale.US, "window.onPhoneLocation(%.7f,%.7f,%.1f)", location.latitude, location.longitude, location.accuracy),
                null,
            )
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, results: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, results)
        if (requestCode == 6 && results.any { it == PackageManager.PERMISSION_GRANTED }) requestPhoneLocation()
        if (requestCode == 9) {
            val granted = results.firstOrNull() == PackageManager.PERMISSION_GRANTED
            if (granted && voiceStartPending) beginVoiceQuestion() else if (!granted) sendVoiceError("未授予麦克风权限，无法语音提问")
            voiceStartPending = false
        }
        if (requestCode == 8) {
            val phone = pendingEmergencyPhone
            pendingEmergencyPhone = null
            if (results.firstOrNull() == PackageManager.PERMISSION_GRANTED && phone != null) {
                sendEmergencySms(phone)
            } else {
                Toast.makeText(this, "未授予短信权限，无法发送跌倒提醒", Toast.LENGTH_LONG).show()
            }
        }
    }

    private fun requestEmergencySms(phone: String) {
        val normalizedPhone = PhoneNumberUtils.stripSeparators(phone)
        if (!PhoneNumberUtils.isGlobalPhoneNumber(normalizedPhone)) {
            Toast.makeText(this, "紧急联系人手机号无效", Toast.LENGTH_LONG).show()
            return
        }
        if (checkSelfPermission(Manifest.permission.SEND_SMS) != PackageManager.PERMISSION_GRANTED) {
            pendingEmergencyPhone = normalizedPhone
            requestPermissions(arrayOf(Manifest.permission.SEND_SMS), 8)
            return
        }
        sendEmergencySms(normalizedPhone)
    }

    private fun sendEmergencySms(phone: String) {
        val location = latestPhoneLocation?.let(::Location)
        Thread {
            val address = location?.let { lookupAddress(it) } ?: "定位暂不可用"
            val coordinates = location?.let {
                String.format(Locale.US, "%.6f,%.6f", it.latitude, it.longitude)
            } ?: "定位暂不可用"
            val message = "【紧急跌倒报警】检测到跌倒，请立即联系并确认安全。地址：$address。坐标：$coordinates。"

            runCatching {
                val manager = SmsManager.getDefault()
                val parts = manager.divideMessage(message)
                if (parts.size > 1) {
                    manager.sendMultipartTextMessage(phone, null, parts, null, null)
                } else {
                    manager.sendTextMessage(phone, null, message, null, null)
                }
            }.onSuccess {
                runOnUiThread { showEmergencySmsSentAlert(phone, coordinates) }
            }.onFailure {
                runOnUiThread { Toast.makeText(this, "短信发送失败：${it.message}", Toast.LENGTH_LONG).show() }
            }
        }.start()
    }

    @Suppress("DEPRECATION")
    private fun lookupAddress(location: Location): String = runCatching {
        if (!Geocoder.isPresent()) return@runCatching "地址解析不可用"
        Geocoder(this, Locale.getDefault()).getFromLocation(location.latitude, location.longitude, 1)
            ?.firstOrNull()?.getAddressLine(0)?.take(80) ?: "地址解析失败"
    }.getOrElse { "地址解析失败" }

    private fun showEmergencySmsSentAlert(phone: String, coordinates: String) {
        if (isFinishing || isDestroyed) return
        getSystemService(Vibrator::class.java)
            ?.vibrate(VibrationEffect.createOneShot(500L, VibrationEffect.DEFAULT_AMPLITUDE))
        AlertDialog.Builder(this)
            .setTitle("⚠ 跌倒报警短信已发送")
            .setMessage("已向 $phone 发送紧急提醒。\n坐标：$coordinates")
            .setPositiveButton("知道了", null)
            .show()
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == 10) {
            val text = if (resultCode == Activity.RESULT_OK) {
                data?.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS)?.firstOrNull()?.trim()
            } else null
            if (text.isNullOrBlank()) sendVoiceError("系统语音识别未返回结果，请检查语音输入服务") else sendVoiceQuestion(text)
            return
        }
        if (requestCode == 7 && resultCode == Activity.RESULT_OK) {
            data?.data?.let { uri -> exportFile?.let { copyCsv(it, uri) } }
        }
        exportFile = null
    }

    private fun copyCsv(source: File, target: Uri) {
        contentResolver.openOutputStream(target)?.use { output -> source.inputStream().use { it.copyTo(output) } }
    }

}
